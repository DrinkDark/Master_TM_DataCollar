/*
 * Copyright (c) 2023 HESSO-VS, HEI Sion
 */

#ifndef _BT_SIG_CTS_H_
#define _BT_SIG_CTS_H_

#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <bluetooth/gatt_dm.h>
#include <bluetooth/services/cts_client.h>

extern bool has_cts;
extern struct bt_cts_client sig_cts_c;

void sig_cts_notify_current_time_cb(struct bt_cts_client *cts_c, struct bt_cts_current_time *current_time);
void sig_cts_enable_notifications(void);
void sig_cts_disable_notification(void);
void sig_cts_discover_completed_cb(struct bt_gatt_dm *dm, void *ctx);
void sig_cts_discover_service_not_found_cb(struct bt_conn *conn, void *ctx);
void sig_cts_discover_error_found_cb(struct bt_conn *conn, int err, void *ctx);

bool sig_cts_read_current_time(struct bt_cts_client *cts_c);

#endif // #ifndef _BT_SIG_CTS_H_
