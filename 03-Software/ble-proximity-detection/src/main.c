/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
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

#define BLE_ADV_INTERVAL_MS 1000			// 1 second
#define BLE_SCAN_WINDOW_MS 2300			// 2.3 seconds
#define BLE_SCAN_INTERVAL_MS 850000 	// 850 seconds
#define BLE_SCAN_INTERVAL_MS_COMPLIANT 10200

static struct k_work scan_work;

int32_t cnt = 0;

uint32_t last_time_ms = 0;

static uint8_t manufacturer[] = {
	0x5A, 0x02, 				// HEI Company Id
	// 0x00, 0x01, 0x00,		// Firmware revision v0.1.0
	0x00,						// Number of days of recording
	0x00						// Status : 0x01 -> waiting for SD Card, 0x02 -> IDLE, 0x03 -> Recording, 0x04 -> Low Batt,  0xff -> Error
};

const static char device_name[] = "BLE detector";
const static uint8_t device_name_len = sizeof(device_name) - 1;

static struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, device_name, device_name_len),
	BT_DATA(BT_DATA_MANUFACTURER_DATA, manufacturer, sizeof(manufacturer))
};

const uint16_t ble_adv_interval = (BLE_ADV_INTERVAL_MS * 1000) / 625;

static struct bt_le_adv_param ble_adv_param[] = {
	BT_LE_ADV_PARAM_INIT(BT_LE_ADV_OPT_CONNECTABLE, ble_adv_interval, ble_adv_interval + 10, NULL)
};

const struct bt_scan_manufacturer_data manufacturer_data = {
	.data = manufacturer,
	.data_len = sizeof(manufacturer)
};

const uint16_t ble_scan_interval = (BLE_SCAN_INTERVAL_MS_COMPLIANT * 1000) / 625;
const uint16_t ble_scan_window = (BLE_SCAN_WINDOW_MS * 1000) / 625;

static struct bt_le_scan_param ble_scan_param[] = {
	BT_LE_SCAN_PARAM_INIT(BT_LE_SCAN_TYPE_PASSIVE, 0, ble_scan_interval, ble_scan_window)
};
static void scan_filter_match(struct bt_scan_device_info *device_info,
			      struct bt_scan_filter_match *filter_match,
			      bool connectable)
{
	uint32_t current_time_ms = k_uptime_get();

	char addr[BT_ADDR_LE_STR_LEN];
	
	bt_addr_le_to_str(device_info->recv_info->addr, addr, sizeof(addr));

	printk("%d,%s,%d,%d\n",
		cnt++, addr, device_info->recv_info->rssi, current_time_ms - last_time_ms);
	
	last_time_ms = current_time_ms;
}

BT_SCAN_CB_INIT(scan_cb, scan_filter_match, NULL,
		NULL, NULL);

static void scan_start(void)
{
	k_work_submit(&scan_work);
}

static int scan_stop(void)
{
	//printk("Scanning stopped ");
	return bt_scan_stop();
}

static void scan_work_handler(struct k_work *work)
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

static void advertising_start(void)
{
	int err = bt_le_adv_start(ble_adv_param, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err) {
		printk("Advertising failed to start (err %d)\n", err);
		return;
	}

	//printk("Advertising started\n");
}

static void advertising_stop(void)
{
	int err = bt_le_adv_stop();
	if (err) {
		printk("Advertising failed to stop (err %d)\n", err);
		return;
	}

	//printk("Advertising stopped\n");
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

	last_time_ms = k_uptime_get();

	printk("Starting Bluetooth peripheral\n");
	//advertising_start();

	k_work_init(&scan_work, scan_work_handler);
	//scan_start();

	printk("Scanning started\n");
	printk("Adv interval : %d ms, scan window : %d ms, scan interval : %d ms\n", (int)(ble_adv_interval * 0.625), (int)(ble_scan_window * 0.625), (int)(ble_scan_interval * 0.625));
	printk("nbr,address,rssi,time\n");

	while (1) {
		advertising_stop();
		scan_start();
		k_sleep(K_MSEC(BLE_SCAN_WINDOW_MS + 200));
		scan_stop();
		advertising_start();
		k_sleep(K_MSEC(BLE_SCAN_INTERVAL_MS - (BLE_SCAN_WINDOW_MS - 2000)));
	}
	

	k_sleep(K_FOREVER);

}
