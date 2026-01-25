/*
 * Copyright (c) 2025, HES-SO Valais-Wallis, HEI, Sion
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>
#include <zephyr/sys/timeutil.h>
#include <bluetooth/scan.h>
#include <zephyr/posix/time.h>
#include <zephyr/sys/byteorder.h>

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

// --- Buffer flush delayable work ---
static struct k_work_delayable proximity_flush_work;

// --- File definitions ---
struct fs_file_t proximity_file;
bool is_proximity_detection_enable;
bool is_proximity_file_opened;

// --- Double buffer system ---
static struct proximity_device_info proximity_store_block[2][CONFIG_BT_PROXIMITY_STORAGE_BUFFER_SIZE];
static uint32_t proximity_sample_offset;
static uint32_t proximity_samples_recorded;
static volatile uint8_t ble_write_idx;
static volatile uint8_t ble_read_idx;

// --- Time gestion ---
static struct tm tm_;
static time_t now;

// --- Single-scan de-duplication ---
#define MAX_DEVICES_PER_SCAN 64
static bt_addr_le_t seen_devices_during_scanning_period[MAX_DEVICES_PER_SCAN];
static uint8_t seen_devices_count = 0;

// --- Bluetooth ---
static uint8_t manufacturer_data[] = {
	0x5A, 0x02, 				// HEI Company Id
	// 0x00, 0x01, 0x00,		// Firmware revision v0.1.0
	0x00,						// Number of days of recording
	0x00						// Status : 0x01 -> waiting for SD Card, 0x02 -> IDLE, 0x03 -> Recording, 0x04 -> Low Batt,  0xff -> Error
};

const struct bt_scan_manufacturer_data scan_manufacturer_data = {
	.data = manufacturer_data,
	.data_len = 2
};

// Use to configure the scan. 10,2s is the maximum scan interval allowed by BLE specification.
// The real scanning interval is handle in ble_thread_init in ble_handler.c
const uint16_t ble_scan_interval = (10200 * 1000) / 625;	
const uint16_t ble_scan_window = (CONFIG_BT_SCAN_WINDOW_MS * 1000) / 625;

static struct bt_le_scan_param ble_scan_param[] = {
	BT_LE_SCAN_PARAM_INIT(BT_LE_SCAN_TYPE_PASSIVE, 0, ble_scan_interval, ble_scan_window)
};

// Scan filter callback
BT_SCAN_CB_INIT(scanning_cb, scanning_filter_match, NULL, NULL, NULL);

/**
 * @brief Initialize the BLE scanning module and internal work queues.
 * 
 * This function registers the BLE scan callbacks and initializes 
 * the asynchronous work items for scanning and buffer timeouts. 
 * It resets all buffer indices and sample counters to zero to 
 * ensure a clean system state.
 * 
 * @param None [in]
 * 
 * @return void
 */

void init_scanning(void)
{
	struct bt_scan_init_param scan_init = {
		.scan_param = &ble_scan_param[0],
		.connect_if_match = false,
	};

	bt_scan_init(&scan_init);

	bt_scan_cb_register(&scanning_cb);

	k_work_init_delayable(&proximity_flush_work, proximity_flush_handler);

	ble_write_idx = 0;
	ble_read_idx = 0;
	proximity_sample_offset = 0;
	proximity_samples_recorded = 0;
	is_proximity_detection_enable = false;
	LOG_DBG("Scan module initialized\n");

}

/**
 * @brief Configures scan filters and starts radio.
 * 
 * Applies Manufacturer Data filters (HEI ID) and activates 
 * passive scanning. This function runs in the system work queue.
 * 
 * @param None [in]
 * 
 * @return int: 0 on success, or negative errno from bt_scan_start.
 *
 */
int start_scanning(void)
{
	 int err;
    static bool filters_initialized = false;
    seen_devices_count = 0;

	// Initialize filter only the first time
    if (!filters_initialized) {
        uint8_t filter_mode = 0;
        
		// Add manufacturer data filter. Filter with HEI BLE identifier (0x5A02)
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

/**
 * @brief Stops the active Bluetooth discovery.
 * 
 * Direct interface to the Zephyr Scan module to stop passive 
 * scanning.
 * 
 * @param None [in]
 * 
 * @return int: 0 on success, or negative errno from bt_scan_stop.
 *
 */

int stop_scanning(void)
{
	int ret = bt_scan_stop();
	if (ret) {
		LOG_ERR("Failed to stop scanning (err %d)\n", ret);
	}

	return ret;
}

/**
 * @brief Parses a numeric ID from an advertisement name.
 * 
 * Uses sscanf with a format that skips non-digit characters 
 * to find the first integer in a string, identifying the peer node.
 * 
 * @param name [in]: Pointer to the string to be parsed.
 *
 * @return int: Extracted ID number, or -1 if no number is found.
 * 
 */
int find_device_number_in_adv_data(const char *name) {
    int number = 0;
    
    if (sscanf(name, "%*[^0123456789]%d", &number) == 1) {
        return number;
    }
    
    return -1; 
}

/**
 * @brief Periodic timeout flush, forces a buffer flush when no more scanning or device detected.
 * 
 * Triggered by a timer. It rotates the buffer and signals the storage 
 * thread to save what has been collected so far.
 * This ensures that the current data isn't trapped in RAM forever.
 * 
 * @param work [in]: Pointer to the k_work_delayable structure.
 * 
 * @return void
 * 
 */
void proximity_flush_handler(struct k_work *work)
{
    if (proximity_sample_offset > 0) {
        LOG_INF("Timeout reached: Flushing %d samples to SD card", proximity_sample_offset);

		// Switch buffer even if not full
        ble_read_idx = ble_write_idx;
		ble_write_idx = (ble_write_idx + 1) & 0x01;
		
		proximity_samples_recorded = proximity_sample_offset;	// Make a copy to use it in the store thread
        proximity_sample_offset = 0;
        k_sem_give(&ble_file_access_sem);
    }
}


/**
 * @brief Callback to parse manufacturer-specific data from advertisement.
 * 
 * This function is called by bt_data_parse() for each advertisement data field.
 * It extracts days_of_recording and system_status from the manufacturer data.
 * 
 * @param data [in]: Pointer to the bt_data structure containing ad type and data
 * @param user_data [in]: User-provided context (pointer to proximity_device_info)
 * 
 * @return bool: true to continue parsing, false to stop
 */
static bool parse_manufacturer_data(struct bt_data *data, void *user_data)
{
    struct proximity_device_info *device = (struct proximity_device_info *)user_data;
    
    if (data->type != BT_DATA_MANUFACTURER_DATA) {
        return true;
    }
    
    // Manufacturer data format:
    // [0-1]: Company ID (0x025A for HEI, little-endian)
    // [2]:   Days of recording
    // [3]:   System status
    
    if (data->data_len < 4) {
        LOG_WRN("Manufacturer data too short: %d bytes", data->data_len);
        return false;
    }
    
    uint16_t company_id = sys_get_le16(&data->data[0]);
    if (company_id != 0x025A) {
        LOG_DBG("Non-HEI company ID: 0x%04X", company_id);
        return false;
    }
    
    device->days_of_recording = data->data[2];
    device->system_status     = data->data[3];
    
    return false;
}

/**
 * @brief Handles successful BLE filter matches.
 * 
 * Filters out devices already seen in the current window. 
 * Copies raw MAC, RSSI, and Device ID into the 13-byte struct. 
 * Signals the store thread when the buffer is full.
 * 
 * @param  device_info [in]: Discovered device metadata (Addr, RSSI).
 * @param  filter_match [in]: Details of which filter was hit.
 * @param  connectable [in]: Connectability status of the peer.
 * 
 * @return void
 * 
 */

static void scanning_filter_match(struct bt_scan_device_info *device_info,
                  struct bt_scan_filter_match *filter_match,
                  bool connectable)
{
	// Single-scan de-duplication
	// Check if we have already seen this device during this scanning period
    for (int i = 0; i < seen_devices_count; i++) {
        if (bt_addr_le_cmp(device_info->recv_info->addr, &seen_devices_during_scanning_period[i]) == 0) {
            return; 
        }
    }

	if(seen_devices_count < MAX_DEVICES_PER_SCAN) {
		bt_addr_le_copy(&seen_devices_during_scanning_period[seen_devices_count++], device_info->recv_info->addr);
	}

	// Copy the device info into the struct
	struct proximity_device_info *device = &proximity_store_block[ble_write_idx][proximity_sample_offset];

	time(&now);
    localtime_r(&now, &tm_);
    device->timestamp = timeutil_timegm(&tm_);  // UTC time in seconds since the Epoch, (1970-01-01 00:00:00 +0000, UTC)
    memcpy(device->addr, device_info->recv_info->addr->a.val, 6);	// Copy raw BLE address
    device->device_number = find_device_number_in_adv_data(device_info->adv_data->data);
	device->rssi = device_info->recv_info->rssi;

    device->days_of_recording = 0;
    device->system_status     = 0xFF;
	bt_data_parse(device_info->adv_data, parse_manufacturer_data, device);

	LOG_DBG("Proximity Device Found: Addr: %s, Device Number: %d, RSSI: %d, Timestamp: %u, Days of Recording: %u, System Status: %u",
			device->addr,
			device->device_number,
			device->rssi,
			device->timestamp,
			device->days_of_recording,
			device->system_status);

	// Enable sound saving when a device is detected near by with a RSSI superior to CONFIG_BT_PROXIMITY_START_SOUND_RSSI_MIN
	#ifdef CONFIG_BT_PROXIMITY_ENABLE_SOUND_SAVING
		if(device->rssi >= CONFIG_BT_PROXIMITY_ENABLE_SOUND_RSSI_MIN) {
			recorder_enable_record_saving();
		}
	#endif //#ifdef CONFIG_BT_PROXIMITY_START_SOUND_SAVING

	// Double buffering and sample writting management
    proximity_sample_offset++;
    if (proximity_sample_offset >= CONFIG_BT_PROXIMITY_STORAGE_BUFFER_SIZE) {
		k_work_cancel_delayable(&proximity_flush_work);		// Delayable work cancel as a new device has been found

		// Rotate buffers
        ble_read_idx = ble_write_idx;
		ble_write_idx = (ble_write_idx + 1) & 0x01;		
		proximity_samples_recorded = proximity_sample_offset;	// Make a copy to use it in the store thread
        proximity_sample_offset = 0;
        k_sem_give(&ble_file_access_sem);
    } else {
		// Buffer is not yet full, reschedule a delayed flush. 
        // This ensures that if no more devices are found or the scanning is stop, the current data isn't trapped in RAM forever
		k_work_reschedule(&proximity_flush_work, K_MINUTES(CONFIG_BT_PROXIMITY_FLUSH_TIMEOUT_MIN));
	}
}


/**
 * @brief Enable proximity detection logic.
 * 
 * Sets the global state flag to true, allowing the scanning 
 * callbacks to begin storing device data into the RAM buffers.
 * 
 * @param None [in]
 * @return void
 * 
 */
void ble_enable_proximity_detection(void)
{
	if (!is_proximity_detection_enable) {
		is_proximity_detection_enable = true;
	}
}

/**
 * @brief Disable proximity detection and close the storage file.
 * 
 * Sets the global state flag to false and performs a safe cleanup by 
 * closing the open SD card file, ensuring FAT entries are updated.
 * 
 * @param None [in]
 * 
 * @return void
 * 
 */
void ble_disable_proximity_detection(void)
{
	if (is_proximity_detection_enable) {
		is_proximity_detection_enable = false;

        proximity_flush_handler(NULL);
	}
}

/**
 * @brief Return proximity detection status.
 * 
 * Return the value of is_proximity_detection_enable.
 * 
 * @param None [in]
 * 
 * @return 1 if proximity detection enabled, 0 if disabled
 * 
 */
bool ble_proximity_is_busy(void)
{
	return is_proximity_detection_enable;
}

/**
 * @brief  Manages SD card storage operations.
 * 
 * Waits for data signals. On the first write, it creates 
 * an 11-byte header. Subsequently, it appends batches of 13-byte 
 * records. Uses fs_sync to protect data against system resets.
 * 
 * @param  None [in]
 * 
 * @return void (Infinite Loop)
 */
void ble_proximity_thread_store_to_file(void)
{
    int w_res;
	time_t end_t;
	double diff_t;

	// Checking if thread could start
	k_sem_take(&thread_proximity_store_busy_sem, K_FOREVER);

	LOG_INF("Proximity store to File Thread for MONKEY application started ...\n");

    while (true) {
        // Wait for a buffer to be full
        k_sem_take(&ble_file_access_sem, K_FOREVER);
		LOG_DBG("ble_file_access_sem taken : %d", ble_file_access_sem.count);

		// Check for power saving mode
        if (must_be_in_power_saving_mode) break;

		k_sem_take(&file_access_sem, K_FOREVER);

        // Open file if not already opened
        if (!is_proximity_file_opened) {
            LOG_WRN("Will open proximity file with index %d", proximity_file_idx);
			
			is_proximity_file_opened = sdcard_file_setup_and_open(&proximity_file, CONFIG_PROXIMITY_STORAGE_FILE_NAME, proximity_file_idx);

			// Add the header to the head of the file
			struct proximity_file_header header;
			header.file_identifier = 0x0B1E;
			header.firmware_version = FIRMWARE_REVISION_NUMBER;
			header.logger_id = flash_device_identifier;

			time(&now);
			localtime_r(&now, &tm_);
			header.start_time = timeutil_timegm(&tm_);

			fs_write(&proximity_file, &header, sizeof(header));
        }

		// Write the data to the current file
        if (is_proximity_file_opened) {
			time(&now);
			localtime_r(&now, &tm_);

			uint8_t idx = ble_read_idx;
			LOG_DBG("Will store to sd card using idx: %d... (ble_read_idx: %d)", idx, ble_read_idx);
			time(&end_t);
			
			uint32_t bytes_to_write = (proximity_samples_recorded * sizeof(struct proximity_device_info));	// Write only recorded samples (proximity_flush_handler() can write buffer not fully filled)

            w_res = fs_write(&proximity_file, proximity_store_block[idx], bytes_to_write);
			LOG_DBG("sdcard_write(...) -> %d, bytes_to_write: %d ", w_res, bytes_to_write);

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
				// Time diff to big, a new file is created
				diff_t = difftime(end_t, start_time_ts.tv_sec);
				LOG_INF("Now: %d.%d.%d %02d:%02d:%02d, Execution time: %.0f [s]", tm_.tm_mday, tm_.tm_mon+1, tm_.tm_year+1900, tm_.tm_hour, tm_.tm_min, tm_.tm_sec, diff_t);
				if (diff_t >= CONFIG_STORAGE_TIME_DIFF_THRESHOLD)
				{
					LOG_WRN("Execution time exceed %d [s]", CONFIG_STORAGE_TIME_DIFF_THRESHOLD);
					sdcard_file_close(&proximity_file);
					is_proximity_file_opened = false;
					time(&start_time_ts.tv_sec);
				}
			}
			fs_sync(&proximity_file);
        }

			// Close file if recording is stopped
		if (!is_proximity_detection_enable && is_proximity_file_opened) {
			LOG_INF("Recording stopped, closing proximity file");
			sdcard_file_close(&proximity_file);
			is_proximity_file_opened = false;
		}

		k_sem_give(&file_access_sem);
    }


	// Giving start's semaphore if thread could start
	k_sem_give(&thread_proximity_store_busy_sem);
	LOG_WRN("------------ Proximity store to File Thread ended ... ------------\n");
}
