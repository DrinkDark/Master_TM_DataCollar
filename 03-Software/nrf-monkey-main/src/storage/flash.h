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
void flash_get_aad_a_params(int *lpf, uint8_t *th);
void flash_get_aad_d1_params(uint8_t *algo, uint16_t *floor, uint16_t *rel_p, uint16_t *abs_p, uint8_t *rel_t, uint16_t *abs_t);
uint32_t flash_get_low_batt_detect_counter(void);
bool     flash_set_low_batt_detect_counter(void);

#endif // #ifndef _FLASH_DEFINE_H_
