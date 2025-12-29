/*
 * Copyright (c) 2023 HESSO-VS, HEI Sion
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
#define LOW_BATT_NODE       			DT_ALIAS(lb0)
#define SD_ENABLE_NODE 					DT_ALIAS(sd0)
#define BURN_COLLAR_NODE				DT_ALIAS(bc0)
#define MIC_CLK_NODE                    DT_ALIAS(clk0)
#define MIC_THSEL_NODE                  DT_ALIAS(thsel0)
#define MIC_WAKE_NODE                   DT_ALIAS(wake0)
#define MIC_ENABLE_NODE					DT_ALIAS(mic0)
#define MIC_OE_NODE						DT_ALIAS(oe0)

#define GPIO0_NODE						DT_NODELABEL(gpio0)
#define GPIO1_NODE						DT_NODELABEL(gpio1)
#define GPIO2_NODE						DT_NODELABEL(gpio2)
#define SPI_NODE						DT_NODELABEL(spi20)
#define I2S_NODE						DT_NODELABEL(i2s20)

extern FATFS main_fat_fs;
extern struct fs_mount_t main_mp;

enum main_state {
	ST_INIT 		= 0x00,
	ST_WAIT_SD_CARD = 0x01,
	ST_IDLE 		= 0x02,
	ST_RECORDING 	= 0x03,
	ST_DISK_FULL	= 0x04,
	ST_LOW_BATT 	= 0x05,
	ST_POWER_SAVING	= 0x06,
	ST_ERROR 		= 0xff
};

extern bool is_sd_gpio_set;
extern bool is_collar_burn_gpio_set;
extern bool is_low_batt_detected;
extern bool ble_open_collar_cmd_received;
extern bool is_saving_enable;

extern struct gpio_dt_spec sd_gpio;

extern struct k_sem low_energy_mode_sem;

extern volatile uint8_t main_state 					__attribute__((section(".noinit")));
extern volatile uint8_t total_days_of_records		__attribute__((section(".noinit")));
extern volatile int recorder_file_idx 				__attribute__((section(".noinit")));
extern volatile int proximity_file_idx				__attribute__((section(".noinit")));
extern volatile uint8_t start_day					__attribute__((section(".noinit")));
extern volatile uint8_t start_month					__attribute__((section(".noinit")));
extern volatile uint8_t hot_reset					__attribute__((section(".noinit")));
extern struct timespec hot_reset_ts					__attribute__((section(".noinit")));
extern struct timespec start_time_ts				__attribute__((section(".noinit")));
extern volatile bool must_be_in_power_saving_mode	__attribute__((section(".noinit")));
extern volatile uint32_t flash_device_identifier	__attribute__((section(".noinit")));
extern volatile int      flash_mic_input_gain		__attribute__((section(".noinit")));
extern volatile int      flash_mic_aad_a_lpf		__attribute__((section(".noinit")));
extern volatile int      flash_mic_aad_a_th			__attribute__((section(".noinit")));

bool open_collar_for_ms(int delay_in_sec);
void set_power_on_sd(bool active);
void set_power_on_mic(bool active);
void enable_output_on_mic(bool active);

void enable_hardware_drivers(void);
void disable_hardware_drivers(void);

void put_device_in_power_save_mode(void);
void system_reset(void);

const char* main_state_to_string(void);


#endif // _MAIN_DEFINE_H_
