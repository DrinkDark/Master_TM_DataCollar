/*
 * Copyright (c) 2023, HES-SO Valais-Wallis, HEI, Sion
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "snes.h"

#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(bt_snes, CONFIG_BT_SNES_LOG_LEVEL);

static struct bt_snes_cb snes_cb;
static uint8_t status_val = 0;
static uint8_t dor_val = 0;
static uint8_t did_val = 0;
static uint8_t mig_val = 0;
static uint16_t maadap_val = 0;
static uint8_t maadd1p_val[10];
static struct bt_conn* conn_handle;

static void snes_ccc_status_changed(const struct bt_gatt_attr* attr, uint16_t value)
{
	LOG_DBG("Notification for `Status` has been turned %s", value == BT_GATT_CCC_NOTIFY ? "ON":"OFF");
    if (snes_cb.status_notif_changed) {
        snes_cb.status_notif_changed(value == BT_GATT_CCC_NOTIFY ? BT_SNES_NOTIFICATION_ENABLED:BT_SNES_NOTIFICATION_DISABLED);
    }
}

static void snes_ccc_dor_changed(const struct bt_gatt_attr* attr, uint16_t value)
{
	LOG_DBG("Notification for `Days of Records` has been turned %s", value == BT_GATT_CCC_NOTIFY ? "ON":"OFF");
    if (snes_cb.dor_notif_changed) {
        snes_cb.dor_notif_changed(value == BT_GATT_CCC_NOTIFY ? BT_SNES_NOTIFICATION_ENABLED:BT_SNES_NOTIFICATION_DISABLED);
    }
}

static void snes_ccc_did_changed(const struct bt_gatt_attr* attr, uint16_t value)
{
	LOG_DBG("Notification for `Device Identifier` has been turned %s", value == BT_GATT_CCC_NOTIFY ? "ON":"OFF");
	if (snes_cb.did_notif_changed) {
        snes_cb.did_notif_changed(value == BT_GATT_CCC_NOTIFY ? BT_SNES_NOTIFICATION_ENABLED:BT_SNES_NOTIFICATION_DISABLED);
	}
}

static void snes_ccc_mig_changed(const struct bt_gatt_attr* attr, uint16_t value)
{
	LOG_DBG("Notification for `Mic Input Gain` has been turned %s", value == BT_GATT_CCC_NOTIFY ? "ON":"OFF");
	if (snes_cb.mig_notif_changed) {
        snes_cb.mig_notif_changed(value == BT_GATT_CCC_NOTIFY ? BT_SNES_NOTIFICATION_ENABLED:BT_SNES_NOTIFICATION_DISABLED);
	}
}

static void snes_ccc_maadap_changed(const struct bt_gatt_attr* attr, uint16_t value)
{
	LOG_DBG("Notification for `Mic AAD A params` has been turned %s", value == BT_GATT_CCC_NOTIFY ? "ON":"OFF");
	if (snes_cb.maadap_notif_changed) {
        snes_cb.maadap_notif_changed(value == BT_GATT_CCC_NOTIFY ? BT_SNES_NOTIFICATION_ENABLED:BT_SNES_NOTIFICATION_DISABLED);
	}
}

static void snes_ccc_maadd1p_changed(const struct bt_gatt_attr* attr, uint16_t value) {
    if (snes_cb.maadd1p_notif_changed)
        snes_cb.maadd1p_notif_changed(value == BT_GATT_CCC_NOTIFY ? BT_SNES_NOTIFICATION_ENABLED : BT_SNES_NOTIFICATION_DISABLED);
}

static ssize_t on_cmd_receive(struct bt_conn* conn, const struct bt_gatt_attr* attr, const void* buf, uint16_t len, uint16_t offset, uint8_t  flags)
{
	LOG_DBG("Received data, handle %d, conn %p", attr->handle, (void *)conn);

	if (snes_cb.cmd_received) {
		snes_cb.cmd_received(conn, buf, len);
    }
	return len;
}

static void on_cmd_sent(struct bt_conn* conn, void* user_data)
{
    ARG_UNUSED(user_data);
    LOG_DBG("New CMD sent, conn %p", (void*) conn);
    if (snes_cb.cmd_sent) {
        snes_cb.cmd_sent(conn);
    }
}

static void on_status_updated(struct bt_conn* conn, void* user_data)
{
    ARG_UNUSED(user_data);
    LOG_DBG("New Status updated, conn %p", (void*) conn);
}

static void on_dor_updated(struct bt_conn* conn, void* user_data)
{
    ARG_UNUSED(user_data);
    LOG_DBG("New Days of Records updated, conn %p", (void*) conn);
}

static void on_did_updated(struct bt_conn* conn, void* user_data)
{
    ARG_UNUSED(user_data);
    LOG_DBG("New Device ID updated, conn %p", (void*) conn);
}

static void on_mig_updated(struct bt_conn* conn, void* user_data)
{
    ARG_UNUSED(user_data);
    LOG_DBG("New Mic Input Gain updated, conn %p", (void*) conn);
}

static void on_maadap_updated(struct bt_conn* conn, void* user_data)
{
    ARG_UNUSED(user_data);
    LOG_DBG("New Mic AAD A params, conn %p", (void*) conn);
}

static void on_maadd1p_updated(struct bt_conn* conn, void* user_data)
{
    ARG_UNUSED(user_data);
    LOG_DBG("New Mic AAD D1 params, conn %p", (void*) conn);
}

static ssize_t read_status(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset)
{
	uint8_t status8 = status_val;
	return bt_gatt_attr_read(conn, attr, buf, len, offset, &status8, sizeof(status8));
}

static ssize_t read_dor(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset)
{
	uint8_t dor8 = dor_val;
	return bt_gatt_attr_read(conn, attr, buf, len, offset, &dor8, sizeof(dor8));
}

static ssize_t read_did(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset)
{
	uint8_t did8 = did_val;
	return bt_gatt_attr_read(conn, attr, buf, len, offset, &did8, sizeof(did8));
}

static ssize_t read_mig(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset)
{
	uint8_t mig8 = mig_val;
	return bt_gatt_attr_read(conn, attr, buf, len, offset, &mig8, sizeof(mig8));
}

static ssize_t read_maadap(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset)
{
	uint16_t maadap16 = maadap_val;
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &maadap16, sizeof(maadap16));
}

static ssize_t read_maadd1p(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset) {
    return bt_gatt_attr_read(conn, attr, buf, len, offset, maadd1p_val, sizeof(maadd1p_val));
}

static int update_char_val(struct bt_conn *conn, const struct bt_gatt_attr* attr, const void *data, uint16_t len, bt_gatt_complete_func_t func)
{
	if (attr) {
		struct bt_gatt_notify_params params = { 0 };
		params.attr = attr;
		params.data = data;
		params.len  = len;
		params.func = func;

		if (!conn) {
			LOG_DBG("Updating the characteristic's value...");
			return bt_gatt_notify_cb(NULL, &params);
		} else if (bt_gatt_is_subscribed(conn, attr, BT_GATT_CCC_NOTIFY)) {
			LOG_DBG("Data send to all connected observers");
			return bt_gatt_notify_cb(conn, &params);
		} else {
			LOG_ERR("Notification NOT enabled for this characteristic...");
			return -EINVAL;
		}
	}

	LOG_ERR("No valid GATT attr !");
	return -ENOENT;
}


/* HSS Service Declaration */
BT_GATT_SERVICE_DEFINE(snes_svc,
	BT_GATT_PRIMARY_SERVICE(BT_UUID_SNES_SERVICE),
	BT_GATT_CHARACTERISTIC(BT_UUID_SNES_CMD, 
					BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
#ifdef CONFIG_BT_SNES_AUTHEN
			    	BT_GATT_PERM_READ_AUTHEN | BT_GATT_PERM_WRITE_AUTHEN,
#else
			    	BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
#endif /* CONFIG_BT_SNES_AUTHEN */
			       	NULL, on_cmd_receive, NULL),
#ifdef CONFIG_BT_USE_USER_DESCRIPTION
	BT_GATT_CUD(BT_CUD_SNES_CMD, BT_GATT_PERM_READ),
#endif
	BT_GATT_CHARACTERISTIC(BT_UUID_SNES_STATUS,
			       	BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
			    	BT_GATT_PERM_READ,
			    	read_status, NULL, &status_val),
	BT_GATT_CUD(BT_CUD_SNES_STATUS, BT_GATT_PERM_READ),
	BT_GATT_CCC(snes_ccc_status_changed,
#ifdef CONFIG_BT_SNES_AUTHEN
			    	BT_GATT_PERM_READ_AUTHEN | BT_GATT_PERM_WRITE_AUTHEN
#else
			    	BT_GATT_PERM_READ | BT_GATT_PERM_WRITE
#endif /* CONFIG_BT_SNES_AUTHEN */
	),
	BT_GATT_CHARACTERISTIC(BT_UUID_SNES_DAYS_OF_RECORDS,
					BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
					BT_GATT_PERM_READ,
			       	read_dor, NULL, &dor_val),
#ifdef CONFIG_BT_USE_USER_DESCRIPTION
	BT_GATT_CUD(BT_CUD_SNES_DAYS_OF_RECORDS, BT_GATT_PERM_READ),
#endif
	BT_GATT_CCC(snes_ccc_dor_changed,
#ifdef CONFIG_BT_SNES_AUTHEN
					BT_GATT_PERM_READ_AUTHEN | BT_GATT_PERM_WRITE_AUTHEN
#else
			    	BT_GATT_PERM_READ | BT_GATT_PERM_WRITE
#endif /* CONFIG_BT_SNES_AUTHEN */
	),

	BT_GATT_CHARACTERISTIC(BT_UUID_SNES_DEVICE_IDENTIFIER,
					BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
					BT_GATT_PERM_READ,
			       	read_did, NULL, &did_val),
#ifdef CONFIG_BT_USE_USER_DESCRIPTION
	BT_GATT_CUD(BT_CUD_SNES_DEVICE_IDENTIFIER, BT_GATT_PERM_READ),
#endif
	BT_GATT_CCC(snes_ccc_did_changed,
#ifdef CONFIG_BT_SNES_AUTHEN
					BT_GATT_PERM_READ_AUTHEN | BT_GATT_PERM_WRITE_AUTHEN
#else
			    	BT_GATT_PERM_READ | BT_GATT_PERM_WRITE
#endif /* CONFIG_BT_SNES_AUTHEN */
	),

	BT_GATT_CHARACTERISTIC(BT_UUID_SNES_MIC_INPUT_GAIN,
					BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
					BT_GATT_PERM_READ,
			       	read_mig, NULL, &mig_val),
#ifdef CONFIG_BT_USE_USER_DESCRIPTION
	BT_GATT_CUD(BT_CUD_SNES_MIC_INPUT_GAIN, BT_GATT_PERM_READ),
#endif
	BT_GATT_CCC(snes_ccc_mig_changed,
#ifdef CONFIG_BT_SNES_AUTHEN
					BT_GATT_PERM_READ_AUTHEN | BT_GATT_PERM_WRITE_AUTHEN
#else
			    	BT_GATT_PERM_READ | BT_GATT_PERM_WRITE
#endif /* CONFIG_BT_SNES_AUTHEN */
	),

	BT_GATT_CHARACTERISTIC(BT_UUID_SNES_MIC_AAD_A_PARAM,
					BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
					BT_GATT_PERM_READ,
			       	read_maadap, NULL, &maadap_val),
#ifdef CONFIG_BT_USE_USER_DESCRIPTION
	BT_GATT_CUD(BT_CUD_SNES_MIC_AAD_A_PARAM, BT_GATT_PERM_READ),
#endif
	BT_GATT_CCC(snes_ccc_maadap_changed,
#ifdef CONFIG_BT_SNES_AUTHEN
					BT_GATT_PERM_READ_AUTHEN | BT_GATT_PERM_WRITE_AUTHEN
#else
			    	BT_GATT_PERM_READ | BT_GATT_PERM_WRITE
#endif /* CONFIG_BT_SNES_AUTHEN */
	),

	BT_GATT_CHARACTERISTIC(BT_UUID_SNES_MIC_AAD_D1_PARAM,
					BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
					BT_GATT_PERM_READ,
			       	read_maadd1p, NULL, &maadd1p_val),
#ifdef CONFIG_BT_USE_USER_DESCRIPTION
	BT_GATT_CUD(BT_CUD_SNES_MIC_AAD_D1_PARAM, BT_GATT_PERM_READ),
#endif
	BT_GATT_CCC(snes_ccc_maadd1p_changed,
#ifdef CONFIG_BT_SNES_AUTHEN
					BT_GATT_PERM_READ_AUTHEN | BT_GATT_PERM_WRITE_AUTHEN
#else
			    	BT_GATT_PERM_READ | BT_GATT_PERM_WRITE
#endif /* CONFIG_BT_SNES_AUTHEN */
	),

);

void on_snes_connected(struct bt_conn* conn)
{
	conn_handle = conn;
}

void on_snes_disconnected(void) 
{ 
	conn_handle = NULL;
}

int bt_snes_init(struct bt_snes_cb *callbacks)
{
	if (callbacks) {
		snes_cb.cmd_received      		= callbacks->cmd_received;
		snes_cb.cmd_sent          		= callbacks->cmd_sent;

		snes_cb.status_notif_changed	= callbacks->status_notif_changed;
		snes_cb.did_notif_changed		= callbacks->did_notif_changed;
		snes_cb.mig_notif_changed		= callbacks->mig_notif_changed;
		snes_cb.maadap_notif_changed	= callbacks->maadap_notif_changed;
		snes_cb.maadd1p_notif_changed	= callbacks->maadd1p_notif_changed;
	}
	return 0;
}

int bt_snes_cmd_send(struct bt_conn *conn, const uint8_t *data, uint16_t len)
{
	const struct bt_gatt_attr* attr = bt_gatt_find_by_uuid(attr_snes_svc, snes_svc.attr_count, BT_UUID_SNES_CMD);
	return update_char_val(conn, attr, data, len, on_cmd_sent);
}

int bt_snes_update_status_cb(uint8_t status)
{
	const struct bt_gatt_attr* attr = bt_gatt_find_by_uuid(attr_snes_svc, snes_svc.attr_count, BT_UUID_SNES_STATUS);

	// Change or Notify characterisitc's value only if it has changes !!!
	if (status != status_val) {
		status_val = status;
		return update_char_val(conn_handle, attr, &status_val, sizeof(status_val), on_status_updated);
	}
	return 0;
}

int bt_snes_update_days_of_records_cb(uint8_t dor)
{
	const struct bt_gatt_attr* attr = bt_gatt_find_by_uuid(attr_snes_svc, snes_svc.attr_count, BT_UUID_SNES_DAYS_OF_RECORDS);

	// Change or Notify characterisitc's value only if it has changes !!!
	if (dor != dor_val) {
		dor_val = dor;
		return update_char_val(conn_handle, attr, &dor_val, sizeof(dor_val), on_dor_updated);
	}
	return 0;
}

int bt_snes_update_device_identifier_cb(uint8_t device_id)
{
	const struct bt_gatt_attr* attr = bt_gatt_find_by_uuid(attr_snes_svc, snes_svc.attr_count, BT_UUID_SNES_DEVICE_IDENTIFIER);

	// Change or Notify characterisitc's value only if it has changes !!!
	if (device_id != did_val) {
		did_val = device_id;
		return update_char_val(conn_handle, attr, &did_val, sizeof(did_val), on_did_updated);
	}
	return 0;
}

int bt_snes_update_mic_input_gain_cb(uint8_t input_gain)
{
	const struct bt_gatt_attr* attr = bt_gatt_find_by_uuid(attr_snes_svc, snes_svc.attr_count, BT_UUID_SNES_MIC_INPUT_GAIN);

	// Change or Notify characterisitc's value only if it has changes !!!
	if (input_gain != mig_val) {
		mig_val = input_gain;
		return update_char_val(conn_handle, attr, &mig_val, sizeof(mig_val), on_mig_updated);
	}
	return 0;
}

int bt_snes_update_aad_a_params_cb(uint8_t input_lpf, uint8_t input_th)
{
    const struct bt_gatt_attr* attr = bt_gatt_find_by_uuid(attr_snes_svc, snes_svc.attr_count, BT_UUID_SNES_MIC_AAD_A_PARAM);

    // Pack LPF into the high byte and Threshold into the low byte
    uint16_t aada_params = (input_lpf << 8) | input_th;

    if (aada_params != maadap_val) {
        maadap_val = aada_params;
        return update_char_val(conn_handle, attr, &maadap_val, sizeof(maadap_val), on_maadap_updated);
    }
    return 0;
}

int bt_snes_update_aad_d1_params_cb(uint8_t algo_sel, uint16_t floor, 
                                  uint16_t rel_pulse, uint16_t abs_pulse, 
                                  uint8_t rel_thr, uint16_t abs_thr)
{
    const struct bt_gatt_attr* attr = bt_gatt_find_by_uuid(attr_snes_svc, snes_svc.attr_count, BT_UUID_SNES_MIC_AAD_D1_PARAM);
    
    uint8_t temp_payload[10];
    temp_payload[0] = algo_sel;
    temp_payload[1] = (uint8_t)(floor & 0xFF);
    temp_payload[2] = (uint8_t)(floor >> 8);
    temp_payload[3] = (uint8_t)(rel_pulse & 0xFF);
    temp_payload[4] = (uint8_t)(rel_pulse >> 8);
    temp_payload[5] = (uint8_t)(abs_pulse & 0xFF);
    temp_payload[6] = (uint8_t)(abs_pulse >> 8);
    temp_payload[7] = (uint8_t)(abs_thr & 0xFF);
    temp_payload[8] = rel_thr;
	temp_payload[9] = (uint8_t)(abs_thr >> 8);

    if (memcmp(temp_payload, maadd1p_val, 10) != 0) {
        memcpy(maadd1p_val, temp_payload, 10);
        return update_char_val(conn_handle, attr, maadd1p_val, 10, NULL);
    }
    return 0;
}