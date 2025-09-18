/*
 * Copyright (c) 2023 HESSO-VS, HEI Sion
 */

#ifndef _GPIO_HAL_DEFINE_H_
#define _GPIO_HAL_DEFINE_H_


void gpio_hal_connect_i2s_gpio(void);

void gpio_hal_connect_spi_gpio(void);

// This function sets all unused GPIO to non pulled, non buffered inputs
void gpio_hal_disconnect_all_unused(void);

void gpio_hal_disconnect_i2s_gpio(void);

void gpio_hal_disconnect_low_batt_gpio(void);

void gpio_hal_disconnect_spi_gpio(void);

#endif // #ifndef _GPIO_HAL_DEFINE_H_