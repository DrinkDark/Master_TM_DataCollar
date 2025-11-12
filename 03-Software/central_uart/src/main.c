/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 *  @brief Nordic UART Service Client sample
 */

#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/printk.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

#include <bluetooth/services/nus.h>
#include <bluetooth/services/nus_client.h>
#include <bluetooth/gatt_dm.h>
#include <bluetooth/scan.h>

#include <zephyr/settings/settings.h>

#include <zephyr/drivers/uart.h>

#include <zephyr/logging/log.h>

#define LOG_MODULE_NAME central_uart
LOG_MODULE_REGISTER(LOG_MODULE_NAME);


static uint8_t manufacturer[] = {
	0x5A, 0x02, 				// HEI Company Id
	// 0x00, 0x01, 0x00,		// Firmware revision v0.1.0
	0x00,						// Number of days of recording
	0x00						// Status : 0x01 -> waiting for SD Card, 0x02 -> IDLE, 0x03 -> Recording, 0x04 -> Low Batt,  0xff -> Error
};

const static char device_name[] = "Central UART";
const static uint8_t device_name_len = sizeof(device_name) - 1;

static struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, device_name, device_name_len),
	BT_DATA(BT_DATA_MANUFACTURER_DATA, manufacturer, sizeof(manufacturer))
};


const struct bt_scan_manufacturer_data manufacturer_data = {
	.data = manufacturer,
	.data_len = sizeof(manufacturer)
};

const uint16_t ble_scan_interval = 1120 * 0.625; // 700 ms
const uint16_t ble_scan_window   = 256 * 0.625; // 160 ms

static struct bt_le_scan_param ble_scan_param[] = {
	BT_LE_SCAN_PARAM_INIT(BT_LE_SCAN_TYPE_PASSIVE, NULL, ble_scan_interval, ble_scan_window)
};
static void scan_filter_match(struct bt_scan_device_info *device_info,
			      struct bt_scan_filter_match *filter_match,
			      bool connectable)
{
	char addr[BT_ADDR_LE_STR_LEN];
	
	bt_addr_le_to_str(device_info->recv_info->addr, addr, sizeof(addr));

	printk("Filters matched. Address: %s rssi: %d\n",
		addr, device_info->recv_info->rssi);
}

BT_SCAN_CB_INIT(scan_cb, scan_filter_match, NULL,
		NULL, NULL);

static int scan_start(void)
{
	int err;
	uint8_t filter_mode = 0;

	err = bt_scan_stop();
	if (err) {
		printk("Failed to stop scanning (err %d)\n", err);
		return err;
	}

	bt_scan_filter_remove_all();

	err = bt_scan_filter_add(BT_SCAN_FILTER_TYPE_MANUFACTURER_DATA, (const void *) &manufacturer_data);
	if (err) {
		printk("Manufacturer data filter cannot be added (err %d)\n", err);
		return err;
	}
	filter_mode |= BT_SCAN_MANUFACTURER_DATA_FILTER;

	err = bt_scan_filter_enable(filter_mode, false);
	if (err) {
		printk("Filters cannot be turned on (err %d)\n", err);
		return err;
	}

	err = bt_scan_start(BT_SCAN_TYPE_SCAN_PASSIVE);
	if (err) {
		printk("Scanning failed to start (err %d)\n", err);
		return err;
	}

	printk("Scan started\n");
	return 0;
}

static void scan_init(void)
{
	struct bt_scan_init_param scan_init = {
		.scan_param = &ble_scan_param[0],
		.connect_if_match = false,
	};

	bt_scan_init(&scan_init);
	bt_scan_cb_register(&scan_cb);

	printk("Scan module initialized\n");
}


int main(void)
{
	int err;
	printk("Starting bluetooth central example\n");

	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return 0;
	}
	printk("Bluetooth initialized\n");

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

	scan_init();
	err = scan_start();
	if (err) {
		return 0;
	}

	printk("Starting Bluetooth central\n");


	for (;;) {
		
	}
}
