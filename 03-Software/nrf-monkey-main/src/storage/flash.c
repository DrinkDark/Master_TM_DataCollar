/*
 * Copyright (c) 2023, HES-SO Valais-Wallis, HEI, Sion
 */

#include "flash.h"

#include <zephyr/drivers/flash.h>
#include <zephyr/logging/log.h>
#include <zephyr/storage/flash_map.h>

LOG_MODULE_REGISTER(flash, CONFIG_FLASH_LOG_LEVEL);

#define STORAGE_NODE_LABEL user_storage

static const struct device* flash_dev;
static struct flash_pages_info fl_info;
static bool is_flash_initialized;
static off_t offset_page_0;
static off_t offset_page_1;
static size_t size_page;

static bool get_uint32_from_flash(off_t offset, uint32_t* value)
{
    if (is_flash_initialized) {
        int rc = flash_read(flash_dev, offset, value, sizeof(uint32_t));
        if (rc == 0) {
            LOG_DBG("value from FLASH: %d (0x%08x)", *value, *value);
            return true;
        } else {
            LOG_WRN("No value found ! (Err: %d)", rc);
        }
    }
    return false;
}

static bool set_uint32_to_flash(off_t offset, size_t size, uint32_t value)
{
    if (is_flash_initialized) {
        uint32_t val = value;
    
        int err = flash_erase(flash_dev, offset, size_page);
        if (err) {
            LOG_ERR("flash_erase(...) FAILED ! Err: %d", err);
        } else {
            err = flash_write(flash_dev, offset, &val, sizeof(uint32_t));
            if (err == 0) {
                LOG_DBG("Value 0x%08x stored in FLASH successfully", val);
                return true;
            } else {
                LOG_ERR("Storage in Flash FAILED ! (err: %d)", err);
            }
        }
    }
    return false;
}

void flash_init(void)
{
    int rc = 0;
    uint32_t flash_offset = 0;
    is_flash_initialized = false;
    
    flash_dev = FLASH_AREA_DEVICE(STORAGE_NODE_LABEL);
    if (!device_is_ready(flash_dev)) {
        LOG_ERR("Flash device %s is not ready!", flash_dev->name);
        return;
    }

    flash_offset    = FLASH_AREA_OFFSET(STORAGE_NODE_LABEL);
    rc              = flash_get_page_info_by_offs(flash_dev, flash_offset, &fl_info);
    if (rc) {
        LOG_ERR("Unable to get page info");
        return;
    }

    offset_page_0   = fl_info.start_offset;
    size_page       = fl_info.size;
    offset_page_1   = flash_offset + size_page;
    LOG_DBG("User FLASH Page 0: Offset=0x%0lx, size=%d", offset_page_0, size_page);
    LOG_DBG("User FLASH Page 1: Offset=0x%08lx, size=%d", offset_page_1, size_page);

    is_flash_initialized = true;
}

uint32_t flash_get_device_identifier(void)
{
    uint32_t result = 0;
    (void) get_uint32_from_flash(offset_page_0, &result);
    return result;
}

int flash_get_mic_input_gain(void)
{
    uint32_t result = 0;
    (void) get_uint32_from_flash(offset_page_0 + 4, &result);
    if (result == 0xffffffff) {
        result = CONFIG_I2S_MIC_INPUT_GAIN;
    }
    return (int) result;
}

uint32_t flash_get_low_batt_detect_counter(void)
{
    uint32_t result = 0;
    (void) get_uint32_from_flash(offset_page_1, &result);
    return result;
}

bool flash_set_low_batt_detect_counter(void)
{
    uint32_t counter = flash_get_low_batt_detect_counter();
    if (counter == 0xffffffff) {
        counter = 0;
    }
    return set_uint32_to_flash(offset_page_1, size_page, ++counter);
}

