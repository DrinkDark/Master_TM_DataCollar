/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <bluetooth/gatt_dm.h>
#include <bluetooth/scan.h>
#include <math.h>

#include <dk_buttons_and_leds.h>

#include <zephyr/settings/settings.h>

#include <zephyr/kernel.h>

// Defines the Semaphores
K_SEM_DEFINE(thread_ble_busy_sem, 1, 1);

#define BLE_SCAN_INTERVAL_MS 2000
#define BLE_SCAN_WINDOW_MS 1000

// Local definitions
static uint8_t manufacturer_data[] = {
	0x5A, 0x02, 				// HEI Company Id
	// 0x00, 0x01, 0x00,		// Firmware revision v0.1.0
	0x00,						
	0x00						
};

const uint16_t ble_scan_interval = (BLE_SCAN_INTERVAL_MS * 1000) / 625;
const uint16_t ble_scan_window = (BLE_SCAN_WINDOW_MS * 1000) / 625;

static struct bt_le_scan_param ble_scan_param[] = {
	BT_LE_SCAN_PARAM_INIT(BT_LE_SCAN_TYPE_PASSIVE, 0, ble_scan_interval, ble_scan_window)
};
int cnt = 0;

bool ble_manufacturer_data_cb(struct bt_data *data, void *user_data)
{
	char *name = user_data;

	switch (data->type) {
    case BT_DATA_MANUFACTURER_DATA:
		memcpy(name, data->data, MIN(data->data_len, 4));
		return false;
	default:
		return true;
	}
}

bool ble_data_cb(struct bt_data *data, void *user_data)
{
	char *name = user_data;

	switch (data->type) {
	case BT_DATA_NAME_SHORTENED:
	case BT_DATA_NAME_COMPLETE:
		memcpy(name, data->data, MIN(data->data_len, 30 - 1));
        name[MIN(data->data_len, 30 - 1)] = '\0';
        return false;
	default:
		return true;
	}
}

void ble_device_found_cb(const bt_addr_le_t *addr, int8_t rssi, uint8_t type, struct net_buf_simple *ad)
{
	
	char addr_str[BT_ADDR_LE_STR_LEN];
    char name[30];
    uint8_t manufacturerData[4];

    memset(name, 0, 30);
    memset(manufacturerData, 0, 4);

    struct net_buf_simple *data1 = malloc(sizeof(struct net_buf_simple));
    struct net_buf_simple *data2 = malloc(sizeof(struct net_buf_simple));

    *data1 = *ad;
    *data2 = *ad;

    // Process the received data to extract the useful information
    bt_data_parse(data1, ble_manufacturer_data_cb, manufacturerData);
    bt_data_parse(data2, ble_data_cb, name);
    bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));
    
    // If the device is a device of interest (manufacturer = 0x5A02 = University of Applied Sciences Valais / 
    // Haute Ecole Valaisanne), add it to the list or udpate if already exist
    if(manufacturerData[0] == 0x5A && manufacturerData[1] == 0x02){
		const double ref_tx_dbm = -40.0;
		const double path_loss_exponent = 2;
		double distance_m = pow(10.0, (ref_tx_dbm - (double)rssi) / (10.0 * path_loss_exponent));
		int int_part = (int)distance_m;
		int frac_part = (int)((distance_m - int_part) * 100);

		printk("%d,%d,%d.%02d\n",
			cnt++, rssi, int_part, frac_part);

    }  

    free(data1);
    free(data2);
}

static bool start_scanning(void)
{
	int err;

	err = bt_le_scan_start(ble_scan_param, ble_device_found_cb);

	if (err) {
		printk("Starting scanning failed (err %d)\n", err);
		return false;
	}
	
	printk("Scanning started successfully\n");
	return true;
}

static bool stop_scanning(void)
{
	int err;

	err = bt_le_scan_stop();
	if (err) {
		printk("Stopping scanning failed (err %d)\n", err);
		return false;
	}

	printk("Scanning stopped successfully\n");
	return true;
}

void main(void)
{
	// ------------------------------------------------------------------------
	// Code for RSSI distance measurement
	// ------------------------------------------------------------------------
	int err;

	printk("Starting RSSI distance measurement central\n");

	// Checking if thread could start
	k_sem_take(&thread_ble_busy_sem, K_FOREVER);

	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	printk("Bluetooth initialized\n");

	k_sleep(K_MSEC(100));  // Give BLE stack time to initialize

	if (!start_scanning()) {
		return;
	}
	printk("meas num,rssi [dBm], distance [m]\n");

	k_sleep(K_FOREVER);

	k_sem_give(&thread_ble_busy_sem);
	printk("------------ BLE Thread ended ! ------------\n");
}