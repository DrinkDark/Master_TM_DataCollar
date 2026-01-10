/*
 * Copyright (c) 2023, HES-SO Valais-Wallis, HEI, Sion
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

#ifdef CONFIG_BT_SNES_SRV
	#include "snes.h"
#endif

#ifdef CONFIG_BT_CTS_CLIENT
	#include "sig_cts.h"
#endif

#ifdef CONFIG_BT_DIS
	#include "sig_dis.h"
#endif

#ifdef CONFIG_BT_PROXIMITY_MONITORING
	#include "ble_proximity.h"
#endif

#include "../audio/recorder.h"
// #include "../storage/flash.h"
#include "../utils/tools.h"
#include "../define.h"


LOG_MODULE_REGISTER(ble_handler, CONFIG_BLE_LOG_LEVEL);

#define DEVICE_NAME 		CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN		(sizeof(DEVICE_NAME) - 1)

// Defines the Semaphores
K_SEM_DEFINE(thread_ble_busy_sem, 1, 1);

// Global variables
struct bt_conn* current_conn;
struct bt_conn* auth_conn;

bool ble_thread_running;
// bool ble_thread_ended;

static bool _advertise_data_changed;

// Local definitions
static uint8_t manufacturer_data[] = {
	0x5A, 0x02, 				// HEI Company Id
	// 0x00, 0x01, 0x00,		// Firmware revision v0.1.0
	0x00,						// Number of days of recording
	0x00						// Status : 0x01 -> waiting for SD Card, 0x02 -> IDLE, 0x03 -> Recording, 0x04 -> Low Batt,  0xff -> Error
};

// Will hold the advertised device's name (ex: Speak No Evil 48)
static char  	device_name[CONFIG_BT_DEVICE_NAME_MAX];
static size_t	device_name_len;

const uint16_t ble_adv_interval = (CONFIG_BT_ADV_INTERVAL_MS * 1000) / 625;

static struct bt_le_adv_param ble_adv_param[] = {
	BT_LE_ADV_PARAM_INIT(BT_LE_ADV_OPT_CONNECTABLE, ble_adv_interval, ble_adv_interval + 10, NULL)
};

#ifdef CONFIG_BT_ADV_SNES_SRV_UUID
static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_SNES_VAL),
};
#endif // #ifdef CONFIG_BT_ADV_SNES_SRV_UUID


static struct bt_le_conn_param bt_conn_param[] = {
	BT_LE_CONN_PARAM_INIT(BT_CONN_INTERVAL_TO_MS(CONFIG_BT_MIN_CONN_INTERVAL), 
						  BT_CONN_INTERVAL_TO_MS(CONFIG_BT_MAX_CONN_INTERVAL), 
						                         CONFIG_BT_SLAVE_LATENCY, 
						   						 CONFIG_BT_SUPERVISION_TIMEOUT * 100),
};

static bool is_low_power_mode_disabled(void)
{
	if (must_be_in_power_saving_mode || is_low_batt_detected) {
		if (must_be_in_power_saving_mode)
			LOG_WRN("Device is in Power Saving mode !");
		else
			LOG_WRN("Device is in Low Batt mode");
		return false;
	}
	return true;
}

#ifdef CONFIG_BT_CTS_CLIENT
	static void discover_completed_cb(struct bt_gatt_dm *dm, void *ctx)
	{
		sig_cts_discover_completed_cb(dm, ctx);

		// Updating connection parameters...
		LOG_DBG("Calling bt_conn_le_param_update(...)");
		int ret = bt_conn_le_param_update(current_conn, bt_conn_param);
		if (ret != 0) {
			LOG_ERR("bt_conn_le_param_update(...) FAILED! Error: %d", ret);
		}
	}

	static void discover_service_not_found_cb(struct bt_conn *conn, void *ctx)
	{
		sig_cts_discover_service_not_found_cb(conn, ctx);
	}

	static void discover_error_found_cb(struct bt_conn *conn, int err, void *ctx)
	{
		sig_cts_discover_error_found_cb(conn, err, ctx);
	}

	// Callback structure for Time Service Client
	static const struct bt_gatt_dm_cb discover_cb = {
		.completed = discover_completed_cb,
		.service_not_found = discover_service_not_found_cb,
		.error_found = discover_error_found_cb,
	};
#endif // #ifdef CONFIG_BT_CTS_CLIENT

static void ble_connected(struct bt_conn* conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	if (err) {
		LOG_ERR("Connection failed (err %u)", err);
		return;
	}

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	current_conn = bt_conn_ref(conn);
	LOG_INF("BLE Connected %s", addr);
	
	if (IS_ENABLED(CONFIG_BT_SNES_SRV))
		on_snes_connected(conn);

	#ifdef CONFIG_BT_CTS_CLIENT
	{
		has_cts = false;
		err = bt_gatt_dm_start(conn, BT_UUID_CTS, &discover_cb, NULL);
		if (err) {
			LOG_ERR("Failed to start discovery (err %d)", err);
		} else {
			LOG_DBG("Starting Time Service discovery on peer device...");
		}
	} 
	#else
	{
		// Updating connection parameters...
		err = bt_conn_le_param_update(conn, bt_conn_param);
		if (err == 0) {
			LOG_INF("Calling bt_conn_le_param_update(...) ...");
		} else {
			LOG_ERR("bt_conn_le_param_update(...) FAILED! Error: %d", err);
		}
	}
	#endif // #ifdef CONFIG_BT_CTS_CLIENT
}

static void ble_disconnected(struct bt_conn *conn, uint8_t reason)
{
	#if (CONFIG_BLE_LOG_LEVEL > 2)
	{
		char addr[BT_ADDR_LE_STR_LEN];
		bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
		LOG_INF("BLE Disconnected: %s (reason %u)", addr, reason);
	}
	#endif // #if (CONFIG_BLE_LOG_LEVEL > 2)

	if (auth_conn) {
		bt_conn_unref(auth_conn);
		auth_conn = NULL;
	}

	if (current_conn) {
		bt_conn_unref(current_conn);
		current_conn = NULL;

		#ifdef CONFIG_BT_SNES_SRV
			on_snes_disconnected();
		#endif // #ifdef CONFIG_BT_SNES_SRV
	}
}

static void ble_param_updated(struct bt_conn * conn, uint16_t interval, uint16_t latency, uint16_t timeout)
{
	LOG_INF("Param updated -> interval: %d, latency: %d, timeout: %d", interval, latency, timeout);

	#ifdef CONFIG_BT_CTS_CLIENT
	{
		// Read current time
		if (sig_cts_read_current_time(&sig_cts_c)) {
			LOG_INF("Reading current time on peer device");
		}
	}
	#endif // #ifdef CONFIG_BT_CTS_CLIENT
}

#ifdef CONFIG_BT_SMP
static void ble_security_changed(struct bt_conn* conn, bt_security_t level, enum bt_security_err err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (err) {
		LOG_WRN("Security failed: %s level %u err %d", addr, level, err);

		#ifdef CONFIG_BT_CTS_CLIENT
		{
			if (err == BT_SECURITY_ERR_AUTH_REQUIREMENT) {
				bt_security_t sec_level = bt_conn_get_security(sig_cts_c.conn);
				err = bt_conn_set_security(sig_cts_c.conn, sec_level | BT_SECURITY_FORCE_PAIR);
				if (err) {
					LOG_ERR("Failed to force security (err %d)", err);
				}
			}
		}
		#endif // #ifdef CONFIG_BT_CTS_CLIENT
	} else {
		LOG_INF("Security changed: %s level %u", addr, level);

		#ifdef CONFIG_BT_CTS_CLIENT
		{
			LOG_DBG("Enabling notification for Time Service ...");
			sig_cts_enable_notifications();
		}
		#endif // #ifdef CONFIG_BT_CTS_CLIENT
	}
}
#endif // #ifdef CONFIG_BT_SMP

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected          = ble_connected,
	.disconnected       = ble_disconnected,
	.le_param_updated	= ble_param_updated,
#ifdef CONFIG_BT_SMP
	.security_changed   = ble_security_changed,
#endif // #ifdef CONFIG_BT_SMP
};


#ifdef CONFIG_BT_SMP
	#ifdef CONFIG_BT_SECURITY_USE_PASSKEY
		static void ble_auth_passkey_display(struct bt_conn *conn, unsigned int passkey)
		{
			char addr[BT_ADDR_LE_STR_LEN];

			bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

			LOG_INF("Passkey for %s: %06u", addr, passkey);
		}

		static void ble_auth_passkey_confirm(struct bt_conn *conn, unsigned int passkey)
		{
			char addr[BT_ADDR_LE_STR_LEN];

			auth_conn = bt_conn_ref(conn);

			bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

			LOG_INF("Passkey for %s: %06u", addr, passkey);
			LOG_INF("Press Button 1 to confirm, Button 2 to reject.");
		}
	#endif // #ifdef CONFIG_BT_SECURITY_USE_PASSKEY

	static void ble_auth_cancel(struct bt_conn *conn)
	{
		char addr[BT_ADDR_LE_STR_LEN];

		bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

		LOG_INF("Pairing cancelled: %s", addr);
	}

	static void ble_pairing_complete(struct bt_conn *conn, bool bonded)
	{
		char addr[BT_ADDR_LE_STR_LEN];

		bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
		LOG_INF("Pairing completed: %s, bonded: %d", addr, bonded);
	}

	static void ble_pairing_failed(struct bt_conn *conn, enum bt_security_err reason)
	{
		char addr[BT_ADDR_LE_STR_LEN];

		bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

		LOG_INF("Pairing failed conn: %s, reason %d", addr, reason);
	}

	static struct bt_conn_auth_cb conn_auth_callbacks = {
		#ifdef CONFIG_BT_SECURITY_USE_PASSKEY
			.passkey_display    = ble_auth_passkey_display,
			.passkey_confirm    = ble_auth_passkey_confirm,
		#endif // #ifdef CONFIG_BT_SECURITY_USE_PASSKEY
		.cancel             	= ble_auth_cancel,
	};

	static struct bt_conn_auth_info_cb conn_auth_info_callbacks = {
		.pairing_complete   	= ble_pairing_complete,
		.pairing_failed     	= ble_pairing_failed
	};
#endif // #ifdef CONFIG_BT_SMP


static void handle_device_name(void)
{
	device_name_len = DEVICE_NAME_LEN;
	memset(device_name, 0, CONFIG_BT_DEVICE_NAME_MAX);
	memcpy(device_name, DEVICE_NAME, DEVICE_NAME_LEN);

	#ifdef CONFIG_DEVICE_IDENTIFIER_FROM_NVS
	{
		// check if the device name in prj.conf has already some digits at its end
		int tmp_len = (DEVICE_NAME_LEN) - 1;
		for (int i = tmp_len; i > 0; i--) {
			if (is_digit(device_name[i]) || device_name[i] == ' ') {
				device_name_len--;
			} else {
				break;
			}
		}
		LOG_DBG("device_name_len:     %d", device_name_len);

		// reset name and reconfigure it
		memset(device_name, 0, CONFIG_BT_DEVICE_NAME_MAX);
		memcpy(device_name, DEVICE_NAME, device_name_len);

		// get device's identifier from FLASH memory
		uint32_t identifier = flash_device_identifier; // flash_get_id_from_flash_memory();
		LOG_DBG("identifier:          %d", identifier);

		int ret = sprintf(&device_name[device_name_len], " %d", (uint8_t)(identifier & 0xff));
		if (ret > 0) {
			device_name_len += ret;
		}
		LOG_DBG("New device_name_len: %d", device_name_len);

		#ifdef CONFIG_BT_DEVICE_NAME_DYNAMIC
		{
			ret = bt_set_name(device_name);
			if (ret) {
				LOG_ERR("Impossible to set the new device name... Err: %d", ret);
			}
		}
		#endif // CONFIG_BT_DEVICE_NAME_DYNAMIC
	}
	#endif // #ifdef CONFIG_DEVICE_IDENTIFIER_FROM_NVS

	LOG_INF("Advertised Device Name: %s (name len: %d)", device_name, device_name_len);
}

static void bt_receive_cb(struct bt_conn* conn, const uint8_t* const data, uint16_t len)
{
	char addr[BT_ADDR_LE_STR_LEN] = {0};

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, ARRAY_SIZE(addr));
	ble_open_collar_cmd_received = false;

    // Decode incoming commands
    if (len > 1 && data[0] == 0xa5) 
	{
		uint8_t cmd = data[1];

		if (cmd == 0x01) {
			LOG_INF("Open Cmd received ...");
			ble_open_collar_cmd_received = true;

			if (is_low_power_mode_disabled()) {
				// Stop recording
				recorder_disable_record();
			}

			// open the locker
			if (open_collar_for_ms(CONFIG_COLLAR_BURN_DELAY_IN_SEC)) {
				LOG_INF("Start burning the nylon wire...");
			}
		} else if (is_low_power_mode_disabled()) {
			switch (cmd)
			{
			case 0x01:
			{
				// already done !!!
				break;
			}
			case 0x02:
			{
				LOG_INF("Reset Device Cmd received ...");
				struct tm tm_;
				clock_gettime(CLOCK_REALTIME, &start_time_ts);
				start_time_ts.tv_nsec = 0;
				time(&start_time_ts.tv_sec);
				localtime_r(&start_time_ts.tv_sec, &tm_);
				LOG_DBG("%02u.%02u.%u %02u:%02u:%02u", tm_.tm_mday, tm_.tm_mon+1, tm_.tm_year + 1900, tm_.tm_hour, tm_.tm_min, tm_.tm_sec);

				start_day = tm_.tm_mday;
				start_month = tm_.tm_mon;
				proximity_file_idx = 1;

				// Enable hardware
				enable_hardware_drivers();

				// start recording
				recorder_enable_record();
				#ifdef CONFIG_BT_PROXIMITY_MONITORING
				{
					ble_enable_proximity_detection();
				}
				#endif // #ifdef CONFIG_BT_PROXIMITY_MONITORING
				break;
			}
			case 0x03:
			{
				LOG_INF("Toggle Recording received ...");
				recorder_disable_record();
				#ifdef CONFIG_BT_PROXIMITY_MONITORING
					ble_disable_proximity_detection();
				#endif // #ifdef CONFIG_BT_PROXIMITY_MONITORING
				break;
			}
			case 0x04:
			{
				LOG_INF("Shutdown HW");
				disable_hardware_drivers();
				break;
			}
			case 0x05:
			{
				LOG_INF("Start up HW");
				enable_hardware_drivers();
				break;
			}
			case 0x06:
			{
				LOG_DBG("Received current time from peer device");
				LOG_HEXDUMP_DBG(data, len, "Received bytes: ");
				if (len == 10) {
					struct tm time;
					time.tm_year    = (((uint16_t)data[3]) << 8) | data[2];
					time.tm_mon		= data[4];
					time.tm_mday	= data[5];
					time.tm_wday    = data[6];
					time.tm_hour	= data[7];
					time.tm_min		= data[8];
					time.tm_sec		= data[9];
					LOG_INF("Peer device time: %02d.%02d.%d %02u:%02u:%02u", time.tm_mday, time.tm_mon+1, time.tm_year + 1900, time.tm_hour, time.tm_min, time.tm_sec);
					time_t now = timeutil_timegm(&time);

					struct timespec ts_now;
					ts_now.tv_sec  = now;
					ts_now.tv_nsec = 0;
					clock_settime(CLOCK_REALTIME, &ts_now);	
				} else {
					LOG_ERR("Data length doesn't match definition !");
				}
				break;
			}
			case 0x07: // Device Identifier
			{
				LOG_DBG("Received new device Identifier from peer device");
				if (len >= 3) {
					if (data[2] != ((uint8_t)(flash_device_identifier & 0xff))) {
						flash_device_identifier = data[2];
						LOG_INF("New flash_device_identifier: %d", flash_device_identifier);
						handle_device_name();
						ble_update_device_id_char_val();
					}
				}
				break;
			}
			case 0x08: // Mic Input Gain
			{
				LOG_DBG("Received new Mic Input Gain params from peer device");
				if (len >= 3) {
					if (data[2] != ((uint8_t)(flash_mic_input_gain & 0xff))) {
						flash_mic_input_gain = data[2];
						LOG_INF("New flash_mic_input_gain: %d", flash_mic_input_gain);
						ble_update_mic_gain_char_val();
					}
				}
				break;
			}
			case 0x09: // Mic AAD A params
			{
				LOG_DBG("Received new Mic AAD A params from peer device");
				// Expecting Packet: [Start 0xA5] [Cmd 0x09] [LPF_VAL] [TH_VAL]
				if (len >= 4) {
					uint8_t new_lpf = data[2];
					uint8_t new_th  = data[3];

					if (new_lpf != flash_mic_aad_a_lpf || new_th != flash_mic_aad_a_th) {
						flash_mic_aad_a_lpf = new_lpf;
						flash_mic_aad_a_th  = new_th;

						LOG_INF("New flash_mic_aad_a_lpf: 0x%02x and new flash_mic_aad_a_th: 0x%02x", (uint8_t)flash_mic_aad_a_lpf, flash_mic_aad_a_th);
						
						ble_update_mic_aada_params_char_val();

						k_sem_give(&reconfig_reset_sem);	// System is reset to apply the new configuration
					}
				} else {
					LOG_ERR("AAD A Command payload too short (len: %d)", len);
				}
				break;
			}
			case 0x0F: // Mic AAD D params
			{
				LOG_DBG("Received new Mic AAD D params from peer device");
				if (len >= 12) {
					uint8_t  n_algo  = data[2];
					uint16_t n_floor = (uint16_t)(data[3] | (data[4] << 8));
					uint16_t n_rel_p = (uint16_t)(data[5] | (data[6] << 8));
					uint16_t n_abs_p = (uint16_t)(data[7] | (data[8] << 8));
					uint8_t  n_rel_t = data[9];
					uint16_t n_abs_t = (uint16_t)(data[10] | (data[11] << 8));
					

					if (n_algo  != flash_mic_aad_d1_algo      ||
						n_floor != flash_mic_aad_d1_floor     ||
						n_rel_p != flash_mic_aad_d1_rel_pulse ||
						n_abs_p != flash_mic_aad_d1_abs_pulse ||
						n_abs_t != flash_mic_aad_d1_abs_thr   ||
						n_rel_t != flash_mic_aad_d1_rel_thr) 
					{
						flash_mic_aad_d1_algo      = n_algo;
						flash_mic_aad_d1_floor     = n_floor;
						flash_mic_aad_d1_rel_pulse = n_rel_p;
						flash_mic_aad_d1_abs_pulse = n_abs_p;
						flash_mic_aad_d1_abs_thr   = n_abs_t;
						flash_mic_aad_d1_rel_thr   = n_rel_t;

						LOG_INF("New flash_mic_aad_d1_algo: 0x%02x, flash_mic_aad_d1_floor: 0x%04x, flash_mic_aad_d1_rel_pulse: 0x%04x, flash_mic_aad_d1_abs_pulse: 0x%04x, flash_mic_aad_d1_rel_thr: 0x%02x, flash_mic_aad_d1_abs_thr: 0x%04x", 
        					 (uint8_t)flash_mic_aad_d1_algo, (uint16_t)flash_mic_aad_d1_floor, 
       						 (uint16_t)flash_mic_aad_d1_rel_pulse, (uint16_t)flash_mic_aad_d1_abs_pulse, 
        					 (uint8_t)flash_mic_aad_d1_rel_thr, (uint16_t)flash_mic_aad_d1_abs_thr);

						bt_snes_update_aad_d1_params_cb(n_algo, n_floor, n_rel_p, n_abs_p, n_abs_t, n_rel_t);
						
    					k_sem_give(&reconfig_reset_sem);	// System is reset to apply the new configuration

					}
				} else {
					LOG_ERR("AAD D Command payload too short (len: %d)", len);
				}
				break;
			}
			case 0xff:
			{
				#ifdef CONFIG_BT_CTS_CLIENT
				{
					LOG_DBG("Disable Notification of Current Time Service ...");
					sig_cts_disable_notification();

					LOG_DBG("Unpairing all devices ...");
					bt_unpair(BT_ID_DEFAULT, BT_ADDR_LE_ANY);

					LOG_DBG("Disconnecting from peer...");
					bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
				}
				#endif // #ifdef CONFIG_BT_CTS_CLIENT
				break;
			}
			default:
				LOG_WRN("Unknown Cmd received ... (cmd: 0x%02x)", cmd);
				break;
			}
		} else {
			LOG_DBG("Skip the command");
		}
    } else {
        LOG_ERR("Invalid data ! len: %d or frame do NOT start with 0xA5 ! (0x%02x)", len, data[0]);
    }

}

static struct bt_snes_cb snes_cb = {
	.cmd_received = bt_receive_cb,
};

static bool start_advertising() 
{
	int err;
	manufacturer_data[2] = total_days_of_records;
	manufacturer_data[3] = main_state;
	struct bt_data ad[] = {
		BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
		BT_DATA(BT_DATA_NAME_COMPLETE, device_name, device_name_len),
		BT_DATA(BT_DATA_MANUFACTURER_DATA, manufacturer_data, sizeof(manufacturer_data))
	};

	#ifdef CONFIG_BT_ADV_SNES_SRV_UUID
		err = bt_le_adv_start(ble_adv_param, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	#else
		err = bt_le_adv_start(ble_adv_param, ad, ARRAY_SIZE(ad), NULL, 0);
	#endif // #ifdef CONFIG_BT_ADV_SNES_SRV_UUID
	if (err) {
		LOG_ERR("Advertising failed to start (err %d)", err);
		return false;
	} else {
		LOG_INF("Is advertising with '%s' (len: %d)", device_name, device_name_len);
	}
	return true;
}

static void stop_advertising(void)
{
	int err = bt_le_adv_stop();
	if (err) {
		LOG_ERR("Advertising failed to stop (err %d)\n", err);
		return;
	}
}

static void set_ble_tx_power(uint8_t handle_type, uint16_t handle, int8_t tx_pwr_level)
{
	struct bt_hci_cp_vs_write_tx_power_level* cp;
	struct bt_hci_rp_vs_write_tx_power_level* rp;
	struct net_buf* buf;
	struct net_buf* resp = NULL;
	int err;

	buf = bt_hci_cmd_create(BT_HCI_OP_VS_WRITE_TX_POWER_LEVEL, sizeof(*cp));
	if (!buf) {
		LOG_ERR("Unable to allocate command buffer");
		return;
	}

	cp = net_buf_add(buf, sizeof(*cp));
	cp->handle = sys_cpu_to_le16(handle);
	cp->handle_type = handle_type;
	cp->tx_power_level = tx_pwr_level;

	err = bt_hci_cmd_send_sync(BT_HCI_OP_VS_WRITE_TX_POWER_LEVEL, buf, &resp);
	if (err) {
		uint8_t reason = resp ? ((struct bt_hci_rp_vs_write_tx_power_level*) resp->data)->status : 0;
		LOG_ERR("Set TX Power FAILED! Error: %d, reason: 0x%02x", err, reason);
		return;
	}

	rp = (void*) resp->data;
	LOG_INF("Current TX Power: %ddBm", rp->selected_tx_power);
	net_buf_unref(resp);
}

void ble_thread_init(void)
{
    int err;

	// RAM initialization
	current_conn					= NULL;
	auth_conn						= NULL;

	ble_thread_running 				= true;
	_advertise_data_changed			= false;

	ble_open_collar_cmd_received	= false;


	// Set up device name with device identifier coming from FLASH or not
	handle_device_name();

	// Checking if thread could start
	k_sem_take(&thread_ble_busy_sem, K_FOREVER);
	
	if (hot_reset != CONFIG_HOT_RESET_VAL) {
		total_days_of_records = 0;
	}

	#ifdef CONFIG_BT_SMP
	{
		err = bt_conn_auth_cb_register(&conn_auth_callbacks);
		if (err) {
			LOG_ERR("Failed to register authorization callbacks.\n");
			return;
		}

		err = bt_conn_auth_info_cb_register(&conn_auth_info_callbacks);
		if (err) {
			LOG_ERR("Failed to register authorization info callbacks.\n");
			return;
		}
	}
	#endif // #ifdef CONFIG_BT_SMP

	#ifdef CONFIG_BT_SNES_SRV
	{
		// SNES Private service initialization
		err = bt_snes_init(&snes_cb);
		if (err) {
			LOG_ERR("Failed to initialize HEI Sync service (err: %d)", err);
			return;
		}
		LOG_INF("HEI Sync service initialized!");
		
		ble_update_device_id_char_val();
		ble_update_mic_gain_char_val();
	}
	#endif // CONFIG_BT_SNES_SRV

	#ifdef CONFIG_BT_CTS_CLIENT
	{
		// Adopted Time Service initialization
		err = bt_cts_client_init(&sig_cts_c);
	}
	#endif // #ifdef CONFIG_BT_CTS_CLIENT

    err = bt_enable(NULL);
    if (err) {
        LOG_ERR("bt_enable() FAILED ! err: %d", err);
    }
    LOG_INF("BLE initialized !");

    if (IS_ENABLED(CONFIG_SETTINGS)) {
        settings_load();
    }

	set_ble_tx_power(BT_HCI_VS_LL_HANDLE_TYPE_ADV, 0, +3);

	#ifdef CONFIG_BT_DIS
	{
		if (!sig_dis_init()) {
			LOG_ERR("sig_dis_init() FAILED !");
		}
	}
	#endif // #ifdef CONFIG_BT_DIS

	#ifdef CONFIG_BT_PROXIMITY_MONITORING
	{
		init_scanning();
	}
	#endif // #ifdef CONFIG_BT_PROXIMITY_MONITORING

	if (!start_advertising()) {
		return;
	}

	#if (CONFIG_BLE_LOG_LEVEL == 4)
	{
		bt_addr_le_t ble_addr;
		struct bt_le_oob ble_oob;
		size_t ble_addr_count = 0;
		char ble_addr_str[BT_ADDR_LE_STR_LEN + 1];

		// Just to check BLE address
		bt_id_get(&ble_addr, &ble_addr_count);
		if (ble_addr_count != 0) {
			memset(ble_addr_str, 0, sizeof(ble_addr_str));
			err = bt_addr_le_to_str(&ble_addr, ble_addr_str, sizeof(ble_addr_str));
			if (err != 0) {
				LOG_DBG("BLE address: %s", ble_addr_str);
			}
		}

		err = bt_le_oob_get_local(BT_ID_DEFAULT, &ble_oob);
		if (err == 0) {
			memset(ble_addr_str, 0, sizeof(ble_addr_str));
			err = bt_addr_le_to_str(&ble_oob.addr, ble_addr_str, sizeof(ble_addr_str));
			if (err != 0) {
				LOG_DBG("BLE oob address: %s", ble_addr_str);
			}
		}
	}
	#endif

	while(ble_thread_running) {
		if (!current_conn && _advertise_data_changed) {
			LOG_WRN("Changing advertise data ! status : %d, days of records: %d", main_state, total_days_of_records);
			_advertise_data_changed = false;
			err = bt_le_adv_stop();

			if (err != 0 || !start_advertising()) {
				return;
			}
		}


		#ifdef CONFIG_BT_PROXIMITY_MONITORING
		{
			// Implements a duty-cycle where the device stops advertising to perform a dedicated passive scan window (need with scan interval greater than 10.23s). 
			// It ensures the SD card is available before starting, as discovered data would otherwise have nowhere to be stored.
			if (sdcard_is_ready()) {
				if (is_proximity_detection_enable) {
					stop_advertising();
					start_scanning();
				
					k_sleep(K_MSEC(CONFIG_BT_SCAN_WINDOW_MS + 100));	// Scan for the duration of the window
				
					stop_scanning();
					start_advertising();

					k_sleep(K_MSEC(CONFIG_BT_SCAN_INTERVAL_MS - (CONFIG_BT_SCAN_WINDOW_MS)));
				} else {
					k_sleep(K_MSEC(200));
				}
			} else {
				k_sleep(K_MSEC(200));
			}
		}	
		#else
		{
			k_sleep(K_MSEC(200));
		}
		#endif // #ifdef CONFIG_BT_PROXIMITY_MONITORING
	}

	LOG_INF("BLE Thread will end soon");

	// Should stop all ble activity
	if (current_conn) {
		bt_conn_disconnect(current_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
	}

	// Stop advertising and scanning
	stop_advertising();

	// Disable BLE Stack
	bt_disable();

	// Giving start's semaphore if thread could start
	k_sem_give(&thread_ble_busy_sem);
	LOG_WRN("------------ BLE Thread ended ! ------------");
}

void ble_update_status_and_dor(uint8_t status, uint8_t nbr)
{
	int ret = bt_snes_update_days_of_records_cb(nbr);
	if (ret == 0) {
		LOG_INF("Nbr of recording's days (%d) notified !", nbr);
	} else if (ret != -ENOTCONN) {
		LOG_DBG("bt_snes_update_days_of_records_cb(%d) FAILED ! err: %d", nbr, ret);
	}

	// Updating Manufacturer Data
	if (main_state != ST_DISK_FULL && main_state != ST_LOW_BATT /* because of the reset for FAT_fs */) {
		if (must_be_in_power_saving_mode) {
			main_state = ST_POWER_SAVING;
		} else {
			main_state = status;
		}
	}

	if (is_low_batt_detected || status == ST_LOW_BATT) {
		main_state = ST_LOW_BATT;
	}

	ret = bt_snes_update_status_cb(main_state);
	if (ret == 0) {
		LOG_INF("New status %02d notified !", main_state);
	} else if (ret != -ENOTCONN) {
		LOG_DBG("bt_snes_update_status_cb(%02d) FAILED ! err: %d", main_state, ret);
	}

	_advertise_data_changed = (manufacturer_data[3] != status) || (manufacturer_data[2] != nbr);
	if (_advertise_data_changed) {
		LOG_INF("New Advertise Data: status: %d, days of records: %d", main_state, nbr);
	}
}

void ble_update_device_id_char_val(void)
{
	int ret = bt_snes_update_device_identifier_cb((uint8_t) (flash_device_identifier & 0xff));
	if (ret) {
		LOG_WRN("Device ID has not been updated...");
	}
}

void ble_update_mic_gain_char_val(void)
{
	int ret = bt_snes_update_mic_input_gain_cb((uint8_t) (flash_mic_input_gain & 0xff));
	if (ret) {
		LOG_WRN("Mic Input Gain has not been updated...");
	}
}

void ble_update_mic_aada_params_char_val(void)
{
    int ret = bt_snes_update_aad_a_params_cb(flash_mic_aad_a_lpf, flash_mic_aad_a_th);
    if (ret) {
        LOG_WRN("Mic AAD A params have not been updated...");
    }
}

void ble_update_mic_aadd1_params_char_val(void)
{
    int ret = bt_snes_update_aad_d1_params_cb(flash_mic_aad_d1_algo, 
											 flash_mic_aad_d1_floor,
											 flash_mic_aad_d1_rel_pulse,
											 flash_mic_aad_d1_abs_pulse,
											 flash_mic_aad_d1_rel_thr,
											 flash_mic_aad_d1_abs_thr);
    if (ret) {
        LOG_WRN("Mic AAD D1 params have not been updated...");
    }
}

void ble_disconnect(void)
{
	int ret = bt_conn_disconnect(current_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
	if (ret) {
		LOG_ERR("bt_conn_disconnect(...) FAILED ! Error: %d", ret);
	}
}