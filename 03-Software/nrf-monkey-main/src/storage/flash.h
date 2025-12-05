/*
 * Copyright (c) 2023 HESSO-VS, HEI Sion
 */

#ifndef _FLASH_DEFINE_H_
#define _FLASH_DEFINE_H_

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

void     flash_init(void);
uint32_t flash_get_device_identifier(void);
int      flash_get_mic_input_gain(void);
uint32_t flash_get_low_batt_detect_counter(void);
bool     flash_set_low_batt_detect_counter(void);

#endif // #ifndef _FLASH_DEFINE_H_
