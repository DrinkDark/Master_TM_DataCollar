/*
 * Copyright (c) 2023, HES-SO Valais-Wallis, HEI, Sion
 */

#include "flash.h"

#include <zephyr/drivers/flash.h>
#include <zephyr/logging/log.h>
#include <zephyr/storage/flash_map.h>

LOG_MODULE_REGISTER(app_flash, CONFIG_FLASH_LOG_LEVEL);

#define STORAGE_PARTITION user_partition

#define STORAGE_PARTITION_OFFSET	FIXED_PARTITION_OFFSET(STORAGE_PARTITION)
#define STORAGE_PARTITION_DEVICE	FIXED_PARTITION_DEVICE(STORAGE_PARTITION)

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
    
    // Use FLASH_AREA_DEVICE with the label string
    flash_dev = STORAGE_PARTITION_DEVICE;;
    if (!device_is_ready(flash_dev)) {
        LOG_ERR("Flash device %s is not ready!", flash_dev->name);
        return;
    }

    // Use FLASH_AREA_OFFSET with the label string
    flash_offset    = STORAGE_PARTITION_OFFSET;
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
    (void)  get_uint32_from_flash(offset_page_0 + 4, &result);
    if (result == 0xffffffff) {
        result = CONFIG_I2S_MIC_INPUT_GAIN;
    }
    return (int) result;
}

void flash_get_aad_a_params(int *lpf, uint8_t *th) {
    uint32_t result = 0;
    (void)  get_uint32_from_flash(offset_page_0 + 4, &result);

    if (result != 0xffffffff) {
        *lpf = (int)(result & 0xFF);
        *th  = (uint8_t)((result >> 8) & 0xFF);
    } else {
        *lpf = CONFIG_MIC_AAD_A_LPF;
        *th  = CONFIG_MIC_AAD_A_TH;
    }
}

void flash_get_aad_d1_params(uint8_t *algo, 
                             uint16_t *floor, 
                             uint16_t *rel_p, 
                             uint16_t *abs_p, 
                             uint8_t *rel_t, 
                             uint16_t *abs_t) {
    uint32_t result1, result2, result3;
    (void)  get_uint32_from_flash(offset_page_0 + 8, &result1);

    (void)  get_uint32_from_flash(offset_page_0 + 12, &result2);

    (void)  get_uint32_from_flash(offset_page_0 + 16, &result3);
    
    if (result1 != 0xffffffff) {
        *algo   = (uint8_t)(result1 & 0xFF);
        *floor  = (uint16_t)(result1 >> 8);
    } else {
        *algo = CONFIG_MIC_AAD_D_ALGO_SEL; 
        *floor = CONFIG_MIC_AAD_D_FLOOR;  
    }

    if (result2 != 0xffffffff) {
        *rel_p  = (uint16_t)(result2 & 0xFFFF);
        *abs_p  = (uint16_t)(result2 >> 16);
    } else {
        *rel_p = CONFIG_MIC_AAD_D_REL_PULSE_MIN; 
        *abs_p = CONFIG_MIC_AAD_D_ABS_PULSE_MIN;
    }

    if (result3 != 0xffffffff) {
        *rel_t  = (uint8_t)(result3 & 0xFF);
        *abs_t  = (uint16_t)(result3 >> 8);
    } else {
        *rel_t = CONFIG_MIC_AAD_D_REL_TH; 
        *abs_t = CONFIG_MIC_AAD_D_ABS_TH; 
    }
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