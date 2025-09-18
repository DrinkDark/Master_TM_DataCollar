/*
 * Copyright (c) 2025 HESSO-VS, HEI Sion
 */

#ifndef _MAIN_DEFINE_H_
#define _MAIN_DEFINE_H_

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/fs/fs.h>
#include <zephyr/posix/time.h>
#include <ff.h>

#include <stdio.h>

// Gpios & Hardware handling
#define SD_MIC_ENABLE_NODE              DT_ALIAS(sm0)

#define GPIO0_NODE                      DT_NODELABEL(gpio0)
#define GPIO1_NODE                      DT_NODELABEL(gpio1)
#define GPIO2_NODE                      DT_NODELABEL(gpio2)
#define SPI_NODE                        DT_NODELABEL(spi20)

#define BTN0_NODE                       DT_NODELABEL(button0)
#define BTN1_NODE                       DT_NODELABEL(button1)
#define BTN2_NODE                       DT_NODELABEL(button2)
#define BTN3_NODE                       DT_NODELABEL(button3)

extern FATFS main_fat_fs;
extern struct fs_mount_t main_mp;
extern struct fs_file_t file;

extern bool is_sd_mic_gpio_set;

extern struct gpio_dt_spec sd_mic_gpio;

extern struct k_sem low_energy_mode_sem;
extern struct k_sem sdcard_activity_sem;

extern volatile uint8_t hot_reset                   __attribute__((section(".noinit")));
extern volatile bool must_be_in_power_saving_mode   __attribute__((section(".noinit")));

void set_power_on_sd_and_mic(bool active);

void enable_hardware_drivers(void);
void disable_hardware_drivers(void);

void system_reset(void);

#endif // _MAIN_DEFINE_H_
