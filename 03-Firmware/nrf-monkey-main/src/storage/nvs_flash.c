/*
 * Copyright (c) 2023, HES-SO Valais-Wallis, HEI, Sion
 */

#include "nvs_flash.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/fs/nvs.h>
#include <zephyr/logging/log.h>
#include <zephyr/storage/flash_map.h>

#include <string.h>

LOG_MODULE_REGISTER(nvs_flash, CONFIG_FLASH_LOG_LEVEL);

static struct nvs_fs fs;
static bool is_nvs_flash_initialized;

#define STORAGE_NODE_LABEL user_storage


static bool get_uint32_from_nvs(uint16_t id, uint32_t* value)
{
    if (is_nvs_flash_initialized) {
        int rc = nvs_read(&fs, id, value, sizeof(uint32_t));
        if (rc > 0) {
            LOG_DBG("value from NVS: %d (0x%08x)", *value, *value);
            return true;
        } else {
            LOG_WRN("No value found ! (Err: %d)", rc);
        }
    }
    return false;
}

static bool set_uint32_to_nvs(uint16_t id, uint32_t value)
{
    if (is_nvs_flash_initialized) {
        uint32_t val = value;
    
        int err = nvs_write(&fs, id, &val, sizeof(uint32_t));
        if (err > 0) {
            LOG_DBG("Value 0x%08x stored in FLASH successfully", val);
            return true;
        } else {
            LOG_ERR("Storage in Flash FAILED ! (err: %d)", err);
        }
    }
    return false;
}


void nvs_flash_init(void)
{
    int rc = 0;
    struct flash_pages_info info;

    is_nvs_flash_initialized = false;

	/* define the nvs file system by settings with:
	 *	sector_size equal to the pagesize,
	 *	3 sectors
	 *	starting anvs_flast FLASH_AREA_OFFSET(storage)
	 */
    fs.flash_device = FLASH_AREA_DEVICE(STORAGE_NODE_LABEL);
	if (!device_is_ready(fs.flash_device)) {
		LOG_ERR("Flash device %s is not ready\n", fs.flash_device->name);
		return;
	}
	fs.offset = FLASH_AREA_OFFSET(STORAGE_NODE_LABEL);
	rc = flash_get_page_info_by_offs(fs.flash_device, fs.offset, &info);
	if (rc) {
		LOG_ERR("Unable to get page info\n");
		return;
	}
	fs.sector_size  = info.size;
	fs.sector_count = FLASH_AREA_SIZE(STORAGE_NODE_LABEL) / info.size ;
	LOG_DBG("Before nvs_init() from app\n");
	rc = nvs_mount(&fs);
	if (rc) {
		LOG_ERR("Flash Init failed\n");
		return;
	}

    LOG_DBG("*** %s partition mounted *** \n", STRINGIFY(STORAGE_NODE_LABEL));
	LOG_DBG("Start address: 0x%x\n", (uint32_t) info.start_offset);
	LOG_DBG("Sectors: %d\n", fs.sector_count);

    is_nvs_flash_initialized = true;

    // For debug
    #ifdef CONFIG_LOG
    {
        uint32_t test_val;
        rc = nvs_read(&fs, 0, &test_val, sizeof(test_val));
        if (rc > 0) {   // item found
            LOG_INF("test_val: 0x%x", test_val);
        } else {        // item not found, add it
            test_val = 0xa6;
            (void) nvs_write(&fs, 0, &test_val, sizeof(test_val));
        }

        rc = nvs_read(&fs, 1, &test_val, sizeof(test_val));
        if (rc > 0) {   // item found
            LOG_INF("test_val: 0x%x", test_val);
        } else {        // item not found, add it
            test_val = 0x00;
            (void) nvs_write(&fs, 1, &test_val, sizeof(test_val));
        }
    }
    #endif // #ifdef CONFIG_LOG

    LOG_INF("+++++++++++++++++ NVS FLASH is initialized ! +++++++++++++++++");
}

uint32_t nvs_flash_get_id_from_flash_memory(void)
{
    uint32_t result = 0;
    (void) get_uint32_from_nvs(0, &result);
    return result;
}

uint32_t nvs_flash_get_low_batt_detect_counter(void)
{
    uint32_t result = 0;
    (void) get_uint32_from_nvs(1, &result);
    return result;
}

bool     nvs_flash_set_low_batt_detect_counter(void)
{
    uint32_t counter = nvs_flash_get_low_batt_detect_counter();
    if (counter == 0xffffffff) {
        counter = 0;
    }
    return set_uint32_to_nvs(1, ++counter);
}
