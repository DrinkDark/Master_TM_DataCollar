/*
 * Copyright (c) 2025, HES-SO Valais-Wallis, HEI, Sion
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/hci_vs.h>
#include <zephyr/bluetooth/gatt.h>
#include <bluetooth/gatt_dm.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/timeutil.h>
#include <bluetooth/scan.h>
#include <zephyr/posix/time.h>

#include <stdio.h>
#include <sys/errno.h>
#include <time.h>

#include "ble_handler.h"
#include "../fatfs/sdcard.h"
#include "../firmware-revision.h"
#include "../audio/recorder.h"

#include "../define.h"

#include "ble_proximity.h"

LOG_MODULE_REGISTER(ble_proximity, CONFIG_BLE_PROXIMITY_LOG_LEVEL);

// Defines the Semaphores
K_SEM_DEFINE(thread_proximity_store_busy_sem, 1, 1);
K_SEM_DEFINE(ble_file_access_sem, 0, 1);

// Local definitions
static uint8_t manufacturer_data[] = {
	0x5A, 0x02, 				// HEI Company Id
	// 0x00, 0x01, 0x00,		// Firmware revision v0.1.0
	0x00,						// Number of days of recording
	0x00						// Status : 0x01 -> waiting for SD Card, 0x02 -> IDLE, 0x03 -> Recording, 0x04 -> Low Batt,  0xff -> Error
};

const struct bt_scan_manufacturer_data scan_manufacturer_data = {
	.data = manufacturer_data,
	.data_len = sizeof(manufacturer_data)
};

static struct k_work ble_scan_work;
static struct k_work_delayable proximity_flush_work;

struct fs_file_t proximity_file;
bool is_proximity_detection_enable;
bool is_proximity_file_opened;

static const uint32_t proximity_block_size = 5 * sizeof(struct proximity_device_info);	// CONFIG_BT_PROXIMITY_STORAGE_BUFFER_SIZE
static struct proximity_device_info proximity_storage_buffers[2][5];	// CONFIG_BT_PROXIMITY_STORAGE_BUFFER_SIZE

static uint8_t proximity_store_idx;
static uint32_t proximity_sample_offset;
static uint32_t proximity_samples_recorded;

static struct tm tm_;
static time_t now;

const uint16_t ble_scan_interval = (10200 * 1000) / 625;	// Maximum scan interval allowed by BLE specification
const uint16_t ble_scan_window = (CONFIG_BT_SCAN_WINDOW_MS * 1000) / 625;

static struct bt_le_scan_param ble_scan_param[] = {
	BT_LE_SCAN_PARAM_INIT(BT_LE_SCAN_TYPE_PASSIVE, 0, ble_scan_interval, ble_scan_window)
};

static inline void proximity_incr_store_idx(void)		{ proximity_store_idx = (proximity_store_idx + 1) & 0x01; }
static inline uint8_t proximity_store_idx_to_write(void)	{ return ((proximity_store_idx - 1) & 0x01); }

#define MAX_DEVICES_PER_SCAN 64
static bt_addr_le_t seen_devices_during_scanning_period[MAX_DEVICES_PER_SCAN];
static uint8_t seen_devices_count = 0;

BT_SCAN_CB_INIT(scanning_cb, scanning_filter_match, NULL, NULL, NULL);

void init_scanning(void)
{
	struct bt_scan_init_param scan_init = {
		.scan_param = &ble_scan_param[0],
		.connect_if_match = false,
	};

	bt_scan_init(&scan_init);

	bt_scan_cb_register(&scanning_cb);

    k_work_init(&ble_scan_work, scanning_work_handler);
	k_work_init_delayable(&proximity_flush_work, proximity_flush_handler);

	proximity_store_idx = 0;
	proximity_sample_offset = 0;
	proximity_samples_recorded = 0;
	is_proximity_detection_enable = false;
	LOG_DBG("Scan module initialized\n");

}

int start_scanning(void)
{
	return k_work_submit(&ble_scan_work);
}

int stop_scanning(void)
{
	int ret = bt_scan_stop();
	if (ret) {
		LOG_ERR("Failed to stop scanning (err %d)\n", ret);
	}

	return ret;
}

int find_device_number_in_adv_data(const char *name) {
    int number = 0;
    
    if (sscanf(name, "%*[^0123456789]%d", &number) == 1) {
        return number;
    }
    
    return -1; 
}

void proximity_flush_handler(struct k_work *work)
{
    if (proximity_sample_offset > 0) {
        LOG_INF("Timeout reached: Flushing %d samples to SD card", proximity_sample_offset);
        proximity_incr_store_idx();	// Switch buffer even if not full
		proximity_samples_recorded = proximity_sample_offset;
        proximity_sample_offset = 0;
        k_sem_give(&ble_file_access_sem);
    }
}

static void scanning_filter_match(struct bt_scan_device_info *device_info,
                  struct bt_scan_filter_match *filter_match,
                  bool connectable)
{
	// Check if we have already seen this device during this scanning period
    for (int i = 0; i < seen_devices_count; i++) {
        if (bt_addr_le_cmp(device_info->recv_info->addr, &seen_devices_during_scanning_period[i]) == 0) {
            return; 
        }
    }

	if(seen_devices_count < MAX_DEVICES_PER_SCAN) {
		bt_addr_le_copy(&seen_devices_during_scanning_period[seen_devices_count++], device_info->recv_info->addr);
	}

	struct proximity_device_info *device = &proximity_storage_buffers[proximity_store_idx][proximity_sample_offset];

    localtime_r(&now, &tm_);
    device->timestamp = timeutil_timegm(&tm_);  // UTC time in seconds since the Epoch, (1970-01-01 00:00:00 +0000, UTC)
    memcpy(device->addr, device_info->recv_info->addr->a.val, 6);	// Copy raw BLE address
    device->device_number = find_device_number_in_adv_data(device_info->adv_data->data);
	device->rssi = device_info->recv_info->rssi;
    
	LOG_DBG("Proximity Device Found: Addr: %s, Device Number: %d, RSSI: %d, Timestamp: %u",
			device->addr,
			device->device_number,
			device->rssi,
			device->timestamp);

	// Enable sound saving when a device is detected near by with a RSSI superior to CONFIG_BT_PROXIMITY_START_SOUND_RSSI_MIN
	#ifdef CONFIG_BT_PROXIMITY_ENABLE_SOUND_SAVING
		if(device->rssi >= CONFIG_BT_PROXIMITY_ENABLE_SOUND_RSSI_MIN) {
			recorder_enable_record_saving();
		}
	#endif //#ifdef CONFIG_BT_PROXIMITY_START_SOUND_SAVING

    proximity_sample_offset++;
    if (proximity_sample_offset >= 5) {	// CONFIG_BT_PROXIMITY_STORAGE_BUFFER_SIZE
		k_work_cancel_delayable(&proximity_flush_work);
        proximity_incr_store_idx();
		proximity_samples_recorded = proximity_sample_offset;
        proximity_sample_offset = 0;
        k_sem_give(&ble_file_access_sem);
    } else {
		k_work_reschedule(&proximity_flush_work, K_MINUTES(CONFIG_BT_PROXIMITY_FLUSH_TIMEOUT_MIN));
	}
}

int scanning_work_handler(struct k_work *work)
{
    int err;
    static bool filters_initialized = false;
    seen_devices_count = 0;

    if (!filters_initialized) {
        uint8_t filter_mode = 0;
        
        err = bt_scan_filter_add(BT_SCAN_FILTER_TYPE_MANUFACTURER_DATA, 
                                (const void *) &scan_manufacturer_data);
        if (err) {
            LOG_ERR("Manufacturer data filter cannot be added (err %d)\n", err);
            return err;
        }
        filter_mode |= BT_SCAN_MANUFACTURER_DATA_FILTER;
        
        err = bt_scan_filter_enable(filter_mode, false);
        if (err) {
            LOG_ERR("Filters cannot be turned on (err %d)\n", err);
            return err;
        }
        
        filters_initialized = true;
    }
    
    err = bt_scan_start(BT_SCAN_TYPE_SCAN_PASSIVE);
    if (err) {
        LOG_ERR("Scanning failed to start (err %d)\n", err);
        return err;
    }

    LOG_DBG("Scanning started\n");
    
    return 0;
}

void ble_enable_proximity_detection(void)
{
	if (!is_proximity_detection_enable) {
		is_proximity_detection_enable = true;
	}
}

void ble_disable_proximity_detection(void)
{
	if (is_proximity_detection_enable) {
		is_proximity_detection_enable = false;

		sdcard_file_close(&proximity_file);
		is_proximity_file_opened = false;
	}
}

void ble_proximity_store_thread(void)
{
    int w_res;
	time_t end_t;
	double diff_t;

	// Checking if thread could start
	k_sem_take(&thread_proximity_store_busy_sem, K_FOREVER);

	LOG_INF("Proximity store to File Thread for MONKEY application started ...\n");

    while (!must_be_in_power_saving_mode) {
        // Wait for a buffer to be full
        k_sem_take(&ble_file_access_sem, K_FOREVER);

        // Open file if not already opened
        if (!is_proximity_file_opened) {
            LOG_WRN("Will open proximity file with index %d", proximity_file_idx);
			
			is_proximity_file_opened = sdcard_file_setup_and_open(&proximity_file, CONFIG_PROXIMITY_STORAGE_FILE_NAME, proximity_file_idx);

			struct proximity_file_header header;
			header.file_identifier = 0x0B1E;
			header.firmware_version = FIRMWARE_REVISION_NUMBER;
			header.logger_id = flash_device_identifier;

			time(&now);
			localtime_r(&now, &tm_);
			header.start_time = timeutil_timegm(&tm_);

			LOG_DBG("Header size : %d", sizeof(header));
			fs_write(&proximity_file, &header, sizeof(header));
        }

        if (is_proximity_file_opened) {
			k_sem_take(&file_access_sem, K_FOREVER);

			time(&now);
			localtime_r(&now, &tm_);

			uint8_t idx = proximity_store_idx_to_write();
			LOG_DBG("Will store to sd card using idx: %d... (proximity_store_idx: %d)", idx, proximity_store_idx);
			time(&end_t);

			uint32_t bytes_to_write = (proximity_samples_recorded * sizeof(struct proximity_device_info));	// Write only recorded samples (proximity_flush_handler() can write buffer not fully filled)

            w_res = fs_write(&proximity_file, proximity_storage_buffers[idx], bytes_to_write);
			LOG_DBG("sdcard_write(...) -> %d, proximity_block_size: %d ", w_res, bytes_to_write);

			proximity_samples_recorded = 0;

			if (w_res == -ENOMEM) {
				// Not enough space on disk => close file and stop recording
				LOG_WRN("SD Card is Full! -> closing file and going to POWER SAVE mode...");
				is_proximity_file_opened = (sdcard_file_close(&proximity_file) != 0);
				ble_update_status_and_dor(ST_DISK_FULL, total_days_of_records);
				put_device_in_power_save_mode();
				return;
			} else if (w_res != bytes_to_write) {
				// Something wrong while writing on SD Card
				LOG_ERR("Nbr bytes written: %d (bytes_to_write: %d)", w_res, bytes_to_write);
				is_proximity_file_opened = (sdcard_file_close(&proximity_file) != 0);
				ble_update_status_and_dor(ST_ERROR, total_days_of_records);
				put_device_in_power_save_mode();
				return;
			} else {
				diff_t = difftime(end_t, start_time_ts.tv_sec);
				LOG_INF("Now: %d.%d.%d %02d:%02d:%02d, Execution time: %.0f [s]", tm_.tm_mday, tm_.tm_mon+1, tm_.tm_year+1900, tm_.tm_hour, tm_.tm_min, tm_.tm_sec, diff_t);
				if (diff_t >= CONFIG_STORAGE_TIME_DIFF_THRESHOLD)
				{
					LOG_WRN("Execution time exceed %d [s]", CONFIG_STORAGE_TIME_DIFF_THRESHOLD);
					sdcard_file_close(&proximity_file);
					is_proximity_file_opened = sdcard_file_setup_and_open(&proximity_file, CONFIG_PROXIMITY_STORAGE_FILE_NAME, ++proximity_file_idx);
					time(&start_time_ts.tv_sec);
				}
			}
			k_sem_give(&file_access_sem);
        }
		
    }
}

// Define the thread
K_THREAD_DEFINE(ble_store_tid, 2048, ble_proximity_store_thread, NULL, NULL, NULL, 6, 0, 0);

