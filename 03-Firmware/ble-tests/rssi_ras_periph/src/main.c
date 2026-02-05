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
#include <math.h>

#include <dk_buttons_and_leds.h>

#include <zephyr/settings/settings.h>

#include <zephyr/kernel.h>

// Defines the Semaphores
K_SEM_DEFINE(thread_ble_busy_sem, 1, 1);

// Local definitions
static uint8_t manufacturer_data[] = {
	0x5A, 0x02, 				// HEI Company Id
	// 0x00, 0x01, 0x00,		// Firmware revision v0.1.0
	0x00,						
	0x00						
};


static struct bt_le_adv_param ble_adv_param[] = {
	BT_LE_ADV_PARAM_INIT(BT_LE_ADV_OPT_CONNECTABLE, BT_GAP_ADV_SLOW_INT_MIN, BT_GAP_ADV_SLOW_INT_MAX, NULL)
};

static bool start_advertising(char *device_name, size_t device_name_len) 
{
	int err;
	struct bt_data ad[] = {
		BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
		BT_DATA(BT_DATA_NAME_COMPLETE, device_name, device_name_len),
		BT_DATA(BT_DATA_MANUFACTURER_DATA, manufacturer_data, sizeof(manufacturer_data))
	};

	err = bt_le_adv_start(ble_adv_param, ad, ARRAY_SIZE(ad), NULL, 0);

	if (err) {
		printk("Advertising failed to start (err %d)", err);
		return false;
	} else {
		printk("Is advertising with '%s' (len: %d)", device_name, device_name_len);
	}
	return true;
}

static bool stop_advertising() 
{
	int err;

	err = bt_le_adv_stop();
	if (err) {
		printk("Failed to stop advertising (err %d)", err);
		return false;
	} else {
		printk("Advertising stopped");
	}
	return true;
}

void main(void)
{
	int err;

	printk("Starting RSSI distance measurement central\n");

	// Checking if thread could start
	k_sem_take(&thread_ble_busy_sem, K_FOREVER);

	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return 0;
	}

	k_sleep(K_MSEC(100));  // Give BLE stack time to initialize

	char device_name[18] = { 'S', 'p', 'e', 'a', 'k', ' ', 'N', 'o', ' ', 'E', 'v', 'i', 'l', ' ', '5', '3'};
	size_t device_name_len = sizeof(device_name)/sizeof(device_name[0]) - 1;

	if (!start_advertising(device_name, device_name_len)) {
		return;
	}
	k_sleep(K_FOREVER);
	
	k_sem_give(&thread_ble_busy_sem);
	printk("------------ BLE Thread ended ! ------------");
}
