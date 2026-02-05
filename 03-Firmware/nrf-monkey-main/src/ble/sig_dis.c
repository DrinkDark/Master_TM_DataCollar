/*
 * Copyright (c) 2023, HES-SO Valais-Wallis, HEI, Sion
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "sig_cts.h"

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/settings/settings.h>

#include <zephyr/kernel.h>

#include "../firmware-revision.h"
#include "../hardware-revision.h"

static int settings_runtime_load(void)
{
	#if defined(CONFIG_BT_DIS_SETTINGS)
	{
		settings_runtime_set("bt/dis/model",
							 CONFIG_BT_DIS_MODEL,
							 sizeof(CONFIG_BT_DIS_MODEL));
		settings_runtime_set("bt/dis/manuf",
							 CONFIG_BT_DIS_MANUF,
							 sizeof(CONFIG_BT_DIS_MANUF));

		#if defined(CONFIG_BT_DIS_SERIAL_NUMBER)
		{
			settings_runtime_set("bt/dis/serial",
								 CONFIG_BT_DIS_SERIAL_NUMBER_STR,
								 sizeof(CONFIG_BT_DIS_SERIAL_NUMBER_STR));
		}
		#endif

		#if defined(CONFIG_BT_DIS_SW_REV)
		{
			settings_runtime_set("bt/dis/sw",
								 CONFIG_BT_DIS_SW_REV_STR,
								 sizeof(CONFIG_BT_DIS_SW_REV_STR));
		}
		#endif

		#if defined(FIRMWARE_REVISION)
		{
			settings_runtime_set("bt/dis/fw",
								 FIRMWARE_REVISION,
								 sizeof(FIRMWARE_REVISION));
		}
		#endif

		#if defined(HARDWARE_REVISION)
		{
			settings_runtime_set("bt/dis/hw",
								 HARDWARE_REVISION,
								 sizeof(HARDWARE_REVISION));
		}
		#endif
	}
	#endif // #if defined(CONFIG_BT_DIS_SETTINGS)
	return 0;
}

bool sig_dis_init(void)
{
	return settings_runtime_load() == 0;
}
