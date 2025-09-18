/*
 * Copyright (c) 2023, HES-SO Valais-Wallis, HEI, Sion
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "sig_cts.h"

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/posix/time.h>
#include <zephyr/sys/timeutil.h>

// #include <stdlib.h>
// #include <sys/time.h>


LOG_MODULE_REGISTER(bt_cts, CONFIG_BT_CTS_LOG_LEVEL);

struct bt_cts_client sig_cts_c;
bool has_cts;

#ifdef CONFIG_BT_CTS_CLIENT
static const char *day_of_week[] = { "Unknown",	  "Monday",   "Tuesday",
				     "Wednesday", "Thursday", "Friday",
				     "Saturday",  "Sunday" };

static const char *month_of_year[] = { "Unknown",   "January", "February",
				       "March",	    "April",   "May",
				       "June",	    "July",    "August",
				       "September", "October", "November",
				       "December" };

static void current_time_print(struct bt_cts_current_time *current_time)
{
	bool ok = true;

	if (current_time->exact_time_256.day == 0) {
		ok = false;
	}

	if (current_time->exact_time_256.year == 0) {
		ok = false;
	}

	LOG_DBG("Adjust reason:");
	LOG_DBG("\tDaylight savings %x", current_time->adjust_reason.change_of_daylight_savings_time);
	LOG_DBG("\tTime zone        %x", current_time->adjust_reason.change_of_time_zone);
	LOG_DBG("\tExternal update  %x", current_time->adjust_reason.external_reference_time_update);
	LOG_DBG("\tManual update    %x", current_time->adjust_reason.manual_time_update);

	if (ok) {
		LOG_DBG("%s, %u %s %u %02u:%02u:%02u", day_of_week[current_time->exact_time_256.day_of_week], current_time->exact_time_256.day, month_of_year[current_time->exact_time_256.month], current_time->exact_time_256.year,
											   current_time->exact_time_256.hours, current_time->exact_time_256.minutes, current_time->exact_time_256.seconds);
	} else {
		LOG_DBG("%s, %u %02u:%02u:%02u", day_of_week[current_time->exact_time_256.day_of_week], current_time->exact_time_256.year,
										 current_time->exact_time_256.hours, current_time->exact_time_256.minutes, current_time->exact_time_256.seconds);
	}

	// Set time in system
	struct tm time;
	time.tm_sec		= current_time->exact_time_256.seconds;
	time.tm_min		= current_time->exact_time_256.minutes;
	time.tm_hour	= current_time->exact_time_256.hours;
	time.tm_mday	= current_time->exact_time_256.day;
	time.tm_mon		= current_time->exact_time_256.month - 1;
	time.tm_year    = current_time->exact_time_256.year - 1900;
	time.tm_wday    = current_time->exact_time_256.day_of_week - 1;
	LOG_DBG("%s, %u %s %u %02u:%02u:%02u", day_of_week[time.tm_wday+1], time.tm_mday, month_of_year[time.tm_mon+1], time.tm_year + 1900, time.tm_hour, time.tm_min, time.tm_sec);
	time_t now = timeutil_timegm(&time);

	struct timespec ts_now;
	ts_now.tv_sec = now;
	clock_settime(CLOCK_REALTIME, &ts_now);	
}

static void read_current_time_cb(struct bt_cts_client *sig_cts_c, struct bt_cts_current_time *current_time, int err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(sig_cts_c->conn), addr, sizeof(addr));
	if (err) {
		LOG_ERR("Cannot read Current Time: %s, error: %d", addr, err);
		return;
	}

	current_time_print(current_time);
}
#endif // CONFIG_BT_CTS_CLIENT

void sig_cts_enable_notifications(void)
{
	#ifdef CONFIG_BT_CTS_CLIENT
	{
		int err;

		#ifdef CONFIG_BT_SMP
		if (has_cts && (bt_conn_get_security(sig_cts_c.conn) >= BT_SECURITY_L2))
		#endif // #ifdef CONFIG_BT_SMP
		{
			err = bt_cts_subscribe_current_time(&sig_cts_c, sig_cts_notify_current_time_cb);
			if (err) {
				LOG_ERR("Cannot subscribe to current time value notification (err %d)", err);
			}
		}
	}
	#endif // #ifdef CONFIG_BT_CTS_CLIENT
}

void sig_cts_disable_notification(void)
{
	#ifdef CONFIG_BT_CTS_CLIENT
	{
		int err = bt_cts_unsubscribe_current_time(&sig_cts_c);
		if (err) {
			LOG_ERR("Cannot unsubscribe to current time value notification (err %d)", err);
		}
	}
	#endif // #ifdef CONFIG_BT_CTS_CLIENT
}

void sig_cts_notify_current_time_cb(struct bt_cts_client *sig_cts_c, struct bt_cts_current_time *current_time)
{
	#ifdef CONFIG_BT_CTS_CLIENT
		current_time_print(current_time);
	#endif // #ifdef CONFIG_BT_CTS_CLIENT
}

void sig_cts_discover_completed_cb(struct bt_gatt_dm *dm, void *ctx)
{
	#ifdef CONFIG_BT_CTS_CLIENT
	{
		int err;

		LOG_DBG("The discovery procedure succeeded");

		bt_gatt_dm_data_print(dm);

		err = bt_cts_handles_assign(dm, &sig_cts_c);
		if (err) {
			LOG_ERR("Could not assign CTS client handles, error: %d", err);
		} else {
			has_cts = true;

			bt_security_t sec_level = bt_conn_get_security(sig_cts_c.conn);
			LOG_DBG("sec_level: %d", sec_level);

			#ifdef CONFIG_BT_SMP
			if (sec_level < BT_SECURITY_L2) {	// Time service needs at least BT_SECURITY_L2
				err = bt_conn_set_security(sig_cts_c.conn, BT_SECURITY_L2 | BT_SECURITY_FORCE_PAIR);
				if (err) {
					LOG_ERR("Failed to set security (err %d)", err);
				}
			} else
			#endif
			{
				LOG_DBG("Enabling notification for Time Service ...");
				sig_cts_enable_notifications();
			}
		}

		err = bt_gatt_dm_data_release(dm);
		if (err) {
			LOG_ERR("Could not release the discovery data, error code: %d\n", err);
		}
	}
	#endif // #ifdef CONFIG_BT_CTS_CLIENT
}

void sig_cts_discover_service_not_found_cb(struct bt_conn *conn, void *ctx)
{
	LOG_WRN("The service could not be found during the discovery");
}

void sig_cts_discover_error_found_cb(struct bt_conn *conn, int err, void *ctx)
{
	LOG_ERR("The discovery procedure failed, err %d", err);
}

bool sig_cts_read_current_time(struct bt_cts_client *cts_c)
{
	#ifdef CONFIG_BT_CTS_CLIENT
	{
		if (has_cts) {
			LOG_DBG("Read current time from Time Service on peer...");
			int err = bt_cts_read_current_time(&sig_cts_c, read_current_time_cb);
			if (err) {
				LOG_ERR("Failed reading current time (err: %d)", err);
				return false;
			}
		} else {
			LOG_ERR("No Time Service discovered !");
		}
	}
	#endif // #ifdef CONFIG_BT_CTS_CLIENT
	return true;
}
