/*
 * Copyright (c) 2023 HESSO-VS, HEI Sion
 */

#include "hss.h"

#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(bt_hss, CONFIG_BT_HSS_LOG_LEVEL);

static struct bt_hss_cb hss_cb;
static uint8_t status_val = 0;
static uint8_t dor_val = 0;
static struct bt_conn* conn_handle;

static void hss_ccc_tx_changed(const struct bt_gatt_attr* attr, uint16_t value)
{
    if (hss_cb.send_enabled) {
        LOG_DBG("Notification for `TX` has been turned %s", value == BT_GATT_CCC_NOTIFY ? "ON":"OFF");
        hss_cb.send_enabled(value == BT_GATT_CCC_NOTIFY ? BT_HSS_SEND_STATUS_ENABLED: BT_HSS_SEND_STATUS_DISABLED);
    }
}

static void hss_ccc_status_changed(const struct bt_gatt_attr* attr, uint16_t value)
{
    if (hss_cb.send_enabled) {
        LOG_DBG("Notification for `Status` has been turned %s", value == BT_GATT_CCC_NOTIFY ? "ON":"OFF");
        hss_cb.send_enabled(value == BT_GATT_CCC_NOTIFY ? BT_HSS_SEND_STATUS_ENABLED: BT_HSS_SEND_STATUS_DISABLED);
    }
}

static void hss_ccc_dor_changed(const struct bt_gatt_attr* attr, uint16_t value)
{
    if (hss_cb.send_enabled) {
        LOG_DBG("Notification for `Days of Records` has been turned %s", value == BT_GATT_CCC_NOTIFY ? "ON":"OFF");
        hss_cb.send_enabled(value == BT_GATT_CCC_NOTIFY ? BT_HSS_SEND_STATUS_ENABLED: BT_HSS_SEND_STATUS_DISABLED);
    }
}

static ssize_t on_receive(struct bt_conn* conn, const struct bt_gatt_attr* attr, const void* buf, uint16_t len, uint16_t offset, uint8_t  flags)
{
	LOG_DBG("Received data, handle %d, conn %p", attr->handle, (void *)conn);

	if (hss_cb.received) {
		hss_cb.received(conn, buf, len);
    }
	return len;
}

static void on_sent(struct bt_conn* conn, void* user_data)
{
    ARG_UNUSED(user_data);
    LOG_DBG("Data send, conn %p", (void*) conn);
    if (hss_cb.sent) {
        hss_cb.sent(conn);
    }
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


/* HSS Service Declaration */
BT_GATT_SERVICE_DEFINE(hss_svc,
BT_GATT_PRIMARY_SERVICE(BT_UUID_HSS_SERVICE),
	BT_GATT_CHARACTERISTIC(BT_UUID_HSS_TX, BT_GATT_CHRC_NOTIFY,
#ifdef CONFIG_BT_HSS_AUTHEN
			    	BT_GATT_PERM_READ_AUTHEN,
#else
			    	BT_GATT_PERM_READ,
#endif /* CONFIG_BT_HSS_AUTHEN */
			       	NULL, NULL, NULL),
#ifdef CONFIG_BT_USE_USER_DESCRIPTION
	BT_GATT_CUD(BT_CUD_HSS_TX, BT_GATT_PERM_READ),
#endif
	BT_GATT_CCC(hss_ccc_tx_changed,
#ifdef CONFIG_BT_HSS_AUTHEN
			    	BT_GATT_PERM_READ_AUTHEN | BT_GATT_PERM_WRITE_AUTHEN),
#else
			    	BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
#endif /* CONFIG_BT_HSS_AUTHEN */
	BT_GATT_CHARACTERISTIC(BT_UUID_HSS_RX, 
					BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
#ifdef CONFIG_BT_HSS_AUTHEN
			    	BT_GATT_PERM_READ_AUTHEN | BT_GATT_PERM_WRITE_AUTHEN,
#else
			    	BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
#endif /* CONFIG_BT_HSS_AUTHEN */
			       	NULL, on_receive, NULL),
#ifdef CONFIG_BT_USE_USER_DESCRIPTION
	BT_GATT_CUD(BT_CUD_HSS_RX, BT_GATT_PERM_READ),
#endif
	BT_GATT_CHARACTERISTIC(BT_UUID_HSS_STATUS,
			       	BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
			    	BT_GATT_PERM_READ,
			    	read_status, NULL, &status_val),
	BT_GATT_CUD(BT_CUD_HSS_STATUS, BT_GATT_PERM_READ),
	BT_GATT_CCC(hss_ccc_status_changed,
#ifdef CONFIG_BT_HSS_AUTHEN
			    	BT_GATT_PERM_READ_AUTHEN | BT_GATT_PERM_WRITE_AUTHEN),
#else
			    	BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
#endif /* CONFIG_BT_HSS_AUTHEN */
	BT_GATT_CHARACTERISTIC(BT_UUID_HSS_DAYS_OF_RECORDS,
					BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
					BT_GATT_PERM_READ,
			       	read_dor, NULL, &dor_val),
#ifdef CONFIG_BT_USE_USER_DESCRIPTION
	BT_GATT_CUD(BT_CUD_HSS_DAYS_OF_RECORDS, BT_GATT_PERM_READ),
#endif
	BT_GATT_CCC(hss_ccc_dor_changed,
#ifdef CONFIG_BT_HSS_AUTHEN
					BT_GATT_PERM_READ_AUTHEN | BT_GATT_PERM_WRITE_AUTHEN),
#else
			    	BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
#endif /* CONFIG_BT_HSS_AUTHEN */
);

void on_hss_connected(struct bt_conn* conn)
{
	conn_handle = conn;
}

void on_hss_disconnected(void) 
{ 
	conn_handle = NULL;
}



int bt_hss_init(struct bt_hss_cb *callbacks)
{
	if (callbacks) {
		hss_cb.received = callbacks->received;
		hss_cb.sent = callbacks->sent;
		hss_cb.send_enabled = callbacks->send_enabled;
	}

	return 0;
}

int bt_hss_send(struct bt_conn *conn, const uint8_t *data, uint16_t len)
{
	struct bt_gatt_notify_params params = { 0 };
	const struct bt_gatt_attr *attr = &hss_svc.attrs[2];

	params.attr = attr;
	params.data = data;
	params.len  = len;
	params.func = on_sent;

	if (!conn) {
		return bt_gatt_notify_cb(NULL, &params);
	} else if (bt_gatt_is_subscribed(conn, attr, BT_GATT_CCC_NOTIFY)) {
		LOG_DBG("Notification send to all connected peers");
		return bt_gatt_notify_cb(conn, &params);
	} else {
		return -EINVAL;
	}
}

int bt_hss_update_status_cb(uint8_t status)
{
	struct bt_gatt_notify_params params = { 0 };
	const struct bt_gatt_attr* attr = bt_gatt_find_by_uuid(attr_hss_svc, hss_svc.attr_count, BT_UUID_HSS_STATUS);
	status_val = status;

	if (attr) {
		params.attr	= attr;
		params.data = &status_val;
		params.len	= sizeof(status_val);
		params.func = on_sent;

		if (!conn_handle) {
			return bt_gatt_notify_cb(NULL, &params);
		} else if (bt_gatt_is_subscribed(conn_handle, attr, BT_GATT_CCC_NOTIFY)) {
			LOG_DBG("Notification send to all connected peers > status: %d", status_val);
			return bt_gatt_notify_cb(conn_handle, &params);
		} else {
			return -EINVAL;
		}
	}
	return -ENOENT;
}

int bt_hss_update_days_of_records_cb(uint8_t dor)
{
	struct bt_gatt_notify_params params = { 0 };
	const struct bt_gatt_attr* attr = bt_gatt_find_by_uuid(attr_hss_svc, hss_svc.attr_count, BT_UUID_HSS_DAYS_OF_RECORDS);
	dor_val = dor;

	if (attr) {
		params.attr	= attr;
		params.data = &dor_val;
		params.len	= sizeof(dor_val);
		params.func = on_sent;

		if (!conn_handle) {
			return bt_gatt_notify_cb(NULL, &params);
		} else if (bt_gatt_is_subscribed(conn_handle, attr, BT_GATT_CCC_NOTIFY)) {
			LOG_DBG("Notification send to all connected peers > nbr of recording days: %d", dor_val);
			return bt_gatt_notify_cb(conn_handle, &params);
		} else {
			return -EINVAL;
		}
	}
	return -ENOENT;
}