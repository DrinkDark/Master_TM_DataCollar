/*
 * Copyright (c) 2023 HESSO-VS, HEI Sion
 */

#ifndef _NVS_FLASH_DEFINE_H_
#define _NVS_FLASH_DEFINE_H_

#include <stdio.h>
#include <stdbool.h>

void     nvs_flash_init(void);
uint32_t nvs_flash_get_id_from_flash_memory(void);
uint32_t nvs_flash_get_low_batt_detect_counter(void);
bool     nvs_flash_set_low_batt_detect_counter(void);

#endif // #ifndef _NVS_FLASH_DEFINE_H_
