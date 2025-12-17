/*
 * Copyright (c) 2023, HES-SO Valais-Wallis, HEI, Sion
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/fs/fs.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/device_runtime.h>
#include <zephyr/posix/time.h>
#include <zephyr/storage/disk_access.h>

#include <errno.h>
#include <ff.h>

#include "define.h"
#include "firmware-revision.h"

#include "audio/recorder.h"
#include "ble/ble_handler.h"
#include "fatfs/sdcard.h"
#include "hal/gpio_hal.h"
#include "storage/flash.h"
#include "microphone/t5848.h"

LOG_MODULE_REGISTER(monkey, CONFIG_MAIN_LOG_LEVEL);

// Define stack size used by thread
#define THREAD_AUDIO_STACKSIZE     		4096
#define THREAD_BLE_STACKSIZE 			CONFIG_BT_SNES_THREAD_STACK_SIZE
#define THREAD_FATFS_STACKSIZE        	4096
#define THREAD_FILE_STACKSIZE        	4096
#define THREAD_MAIN_STACKSIZE     		4096

// Set the priority of the thread
#define THREAD_AUDIO_PRIORITY      		6
#define THREAD_BLE_PRIORITY 			6
#define THREAD_FATFS_PRIORITY       	7
#define THREAD_FILE_PRIORITY       		7
#define THREAD_MAIN_PRIORITY       		5

// Gpios & Hardware handling
#if !DT_NODE_HAS_STATUS(SPI_NODE, okay)
	#warning "SPI Node is NOT okay"
#else
	static const struct device* spi_dev = DEVICE_DT_GET(SPI_NODE);
#endif

#if !DT_NODE_HAS_STATUS(I2S_NODE, okay)
	#warning "I2S Node is NOT okay"
#else
	static const struct device* i2s_dev = DEVICE_DT_GET(I2S_NODE);
#endif

// FAT_fs mounting info
FATFS main_fat_fs;
struct fs_mount_t main_mp = {
	.type = FS_FATFS,
	.fs_data = &main_fat_fs,
};

volatile uint8_t main_state;
volatile uint8_t total_days_of_records;
volatile int file_idx;
volatile uint8_t start_day;
volatile uint8_t start_month;
volatile uint8_t hot_reset;
volatile bool must_be_in_power_saving_mode;
volatile uint32_t flash_device_identifier;
volatile int flash_mic_input_gain;

time_t hot_reset_time;
struct timespec hot_reset_ts;
struct timespec start_time_ts;

bool is_sd_gpio_set;
bool is_mic_set;
bool is_collar_burn_gpio_set;
bool is_low_batt_detected;
bool ble_open_collar_cmd_received;

bool is_main_thread_initialized;

// Low Batt GPIO handlers
#if DT_NODE_HAS_STATUS(LOW_BATT_NODE, okay)
	static struct gpio_dt_spec low_batt_gpio = GPIO_DT_SPEC_GET(LOW_BATT_NODE, gpios);
	static struct k_work_delayable low_batt_work;
	K_SEM_DEFINE(low_energy_mode_sem, 1, 1);

	static void low_batt_debounced(struct k_work* work)
	{
		ARG_UNUSED(work);
		if (gpio_pin_get_dt(&low_batt_gpio) != 0)
		{
			LOG_WRN("Low Batt mode detected!");
			is_low_batt_detected = true;

			LOG_DBG("Disconnect Low Batt GPIO...");
			gpio_hal_disconnect_low_batt_gpio();

			LOG_DBG("Updating device status...");
			ble_update_status_and_dor(ST_LOW_BATT, total_days_of_records);

			LOG_DBG("Give `low_energy_mode_sem` ...");
			k_sem_give(&low_energy_mode_sem);
		}
		else
		{
			LOG_INF("Not in Low Batt mode");
			k_sem_take(&low_energy_mode_sem, K_NO_WAIT);
			is_low_batt_detected = false;
		}
	}
	static K_WORK_DELAYABLE_DEFINE(low_batt_work, low_batt_debounced);

	void low_batt_handler(const struct device* dev, struct gpio_callback* cb, uint32_t pins)
	{
		// Storing in FLASH the number of low batt detection...
		if (gpio_pin_get_dt(&low_batt_gpio) != 0) {
			flash_set_low_batt_detect_counter();
		}

		// run debouncer ...
		if (CONFIG_LOW_BATT_GPIO_DEBOUNCE_MS != 0) {
			k_work_reschedule(&low_batt_work, K_MSEC(CONFIG_LOW_BATT_GPIO_DEBOUNCE_MS));
		} else {
			low_batt_debounced(NULL);
		}
	}

	static bool init_low_batt_gpio(void)
	{
		static bool low_batt_gpio_init = true;
		static struct gpio_callback low_batt_callback;
		int ret;
		if (low_batt_gpio_init)
		{
			if (!device_is_ready(low_batt_gpio.port)) 
			{
				LOG_ERR("%s is not ready", low_batt_gpio.port->name);
				return false;
			}

			ret = gpio_pin_configure_dt(&low_batt_gpio, GPIO_INPUT);
			if (ret < 0) 
			{
				LOG_ERR("Failed to configure %s pin %d: %d", low_batt_gpio.port->name, low_batt_gpio.pin, ret);
				return false;
			}

			ret = gpio_pin_interrupt_configure_dt(&low_batt_gpio, GPIO_INT_TRIG_BOTH);
			if (ret < 0) 
			{
				LOG_ERR("Failed to configure interrupt on %s pin %d: %d", low_batt_gpio.port->name, low_batt_gpio.pin, ret);
				return false;
			}

			gpio_init_callback(&low_batt_callback, low_batt_handler, BIT(low_batt_gpio.pin));
			gpio_add_callback(low_batt_gpio.port, &low_batt_callback);

			// Wait some time here to let hardware initialize correctly !!!
			k_msleep(CONFIG_START_UP_DELAY_MS);
			low_batt_debounced(NULL);
			LOG_INF("GPIO %d configuration completed", low_batt_gpio.pin);
			low_batt_gpio_init = false;
		}
		return true;
	}
#else
	struct gpio_dt_spec low_batt_gpio = NULL;
	static bool init_low_batt_gpio(void) { return true; }
#endif // #if DT_NODE_HAS_STATUS(LOW_BATT_NODE, okay)

// SD Card Power GPIO handlers
#if DT_NODE_HAS_STATUS(SD_ENABLE_NODE, okay)
	struct gpio_dt_spec sd_gpio = GPIO_DT_SPEC_GET(SD_ENABLE_NODE, gpios);

	static bool init_sd_gpio(void)
	{
		static bool sd_gpio_init = true;
		int ret;
		if (sd_gpio_init)
		{
			if (!device_is_ready(sd_gpio.port)) 
			{
				LOG_ERR("%s is not ready", sd_gpio.port->name);
				return false;
			}

			ret = gpio_pin_configure_dt(&sd_gpio, GPIO_OUTPUT_INACTIVE /* GPIO_OUTPUT_ACTIVE */);
			if (ret < 0) 
			{
				LOG_ERR("Failed to configure %s pin %d: %d", sd_gpio.port->name, sd_gpio.pin, ret);
				return false;
			}
			LOG_INF("GPIO %d configuration completed", sd_gpio.pin);
			is_sd_gpio_set = false; //true;
			sd_gpio_init = false;
			disable_hardware_drivers();

			#if !DT_NODE_HAS_STATUS(I2S_NODE, okay)
			{
				gpio_hal_disconnect_i2s_gpio();
			}
			#endif

			#if !DT_NODE_HAS_STATUS(SPI_NODE, okay)
			{
				gpio_hal_disconnect_spi_gpio();
			}
			#endif

		}
		return true;
	}
#else
	struct gpio_dt_spec sd_gpio = NULL;
	static bool init_sd_gpio(void) { return true; }
#endif // #if DT_NODE_HAS_STATUS(SD_ENABLE_NODE, okay)

// Collar Burner GPIO handlers
#if DT_NODE_HAS_STATUS(BURN_COLLAR_NODE, okay)
	struct gpio_dt_spec collar_burn_gpio = GPIO_DT_SPEC_GET(BURN_COLLAR_NODE, gpios);
	static struct k_work_delayable burn_collar_release_work;
    
	static bool init_burn_collar_gpio(void)
	{
		static bool burn_collar_gpio_init = true;
		int ret;
		if (burn_collar_gpio_init)
		{
			if (!device_is_ready(collar_burn_gpio.port)) 
			{
				LOG_ERR("%s is not ready", collar_burn_gpio.port->name);
				return false;
			}

			ret = gpio_pin_configure_dt(&collar_burn_gpio, GPIO_OUTPUT_INACTIVE);
			if (ret < 0) 
			{
				LOG_ERR("Failed to configure %s pin %d: %d", collar_burn_gpio.port->name, collar_burn_gpio.pin, ret);
				return false;
			}
			LOG_INF("GPIO %d configuration completed", collar_burn_gpio.pin);
			is_collar_burn_gpio_set = false;
			burn_collar_gpio_init = false;
		}
		return true;
	}

	static void collar_burn_release(struct k_work* work)
	{
		ARG_UNUSED(work);
		int ret = gpio_pin_set_dt(&collar_burn_gpio, 0);
		if (ret == 0) {
			is_collar_burn_gpio_set = false;
			LOG_WRN("Stop nylon wire burning...");
		} else {
			LOG_ERR("gpio_pin_set_dt(&collar_burn_gpio, 0) FAILED ! Error: %d", ret);
		}

		LOG_DBG("Will disconnect from collar ...");
		ble_disconnect();

		LOG_DBG("Ending all threads except BLE ...");
		put_device_in_power_save_mode();
	}
	
	static K_WORK_DELAYABLE_DEFINE(burn_collar_release_work, collar_burn_release);
	bool open_collar_for_ms(int delay_in_sec)
	{
		int ret = gpio_pin_set_dt(&collar_burn_gpio, 1);
		if (ret == 0) {
			LOG_INF("Start burining nylon wire ...");
			is_collar_burn_gpio_set = true;
			int res = k_work_reschedule(&burn_collar_release_work, K_SECONDS(delay_in_sec));
			if (res == 0 || res == 1) {
				LOG_DBG("burn_collar_release_work scheduled successfully in %d sec.", delay_in_sec);
				return true;
			} else {
				LOG_ERR("k_work_reschedule(...) FAILED ! Error: %d", res);
			}
		} else {
			LOG_ERR("gpio_pin_set_dt(&collar_burn_gpio, 1) FAILED ! Error: %d", ret);
		}
		return false;
	}
#else
	struct gpio_dt_spec collar_burn_gpio = NULL;
	bool open_collar_for_ms(int delay_in_sec) 		{ return true; }
	void collar_burn_release(struct k_work* work) 	{ return true; }
#endif // #if DT_NODE_HAS_STATUS(BURN_COLLAR_NODE, okay)

// Mic GPIO handlers
#if DT_NODE_HAS_STATUS(MIC_WAKE_NODE, okay) & DT_NODE_HAS_STATUS(MIC_ENABLE_NODE, okay) & DT_NODE_HAS_STATUS(MIC_OE_NODE, okay)
	struct gpio_dt_spec mic_wake_gpio = GPIO_DT_SPEC_GET(MIC_WAKE_NODE, gpios);
	struct gpio_dt_spec mic_enable_gpio = GPIO_DT_SPEC_GET(MIC_ENABLE_NODE, gpios);
	struct gpio_dt_spec mic_oe_gpio = GPIO_DT_SPEC_GET(MIC_OE_NODE, gpios);
	static struct gpio_callback mic_wake_cb_data;
	struct k_work_delayable mic_head_recording_work;
	struct k_work_delayable mic_tail_recording_work;

	bool mic_init_done;
	uint32_t pulse_start_cycle;

	K_SEM_DEFINE(mic_config_done_sem, 0, 1);

	void mic_wake_handler(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
		uint32_t now = k_cycle_get_32();

		// Control confirmation pulse after configuration
		if (!mic_init_done) {
			if (gpio_pin_get_dt(&mic_wake_gpio) == 1) {
				pulse_start_cycle = now;
			} else {
				uint32_t pulse_stop_cycle = now;

				if (pulse_start_cycle == 0) {
					return; 
				}

				uint32_t duration_cycles = now - pulse_start_cycle;
				uint32_t duration_us = k_cyc_to_us_near32(duration_cycles);
				pulse_start_cycle = 0; 

				if (duration_us >= 8 && duration_us <= 16) {
					is_mic_set = true;
					mic_init_done = true;
					k_sem_give(&mic_config_done_sem);
					LOG_INF("Microphone has received config !");
				} else {
					LOG_WRN("Pulse invalid! Expected ~12us, got %d us", duration_us);
				}
			}
		} else {
			if(k_sem_count_get(&recorder_toggle_transfer_sem) == 0) {
				if (gpio_pin_get_dt(&mic_wake_gpio) == 1) {
					//Debounce the rising edge
					if ((CONFIG_MIC_RECORDING_HEAD_DEBOUNCE_MSEC) != 0) {
						k_work_reschedule(&mic_head_recording_work, K_MSEC(CONFIG_MIC_RECORDING_HEAD_DEBOUNCE_MSEC));
					} else {
						mic_start_recording(NULL);
					}

				} else {
					// Record for X mS after the falling edge (CONFIG_MIC_RECORDING_TAIL_MSEC)
					// Also acts as a debouncer, this prevents recording interruptions when there are 
					// short interruptions in the wake signal (low state duration < CONFIG_MIC_RECORDING_TAIL_MSEC)
					if ((CONFIG_MIC_RECORDING_TAIL_MSEC) != 0) {
						k_work_reschedule(&mic_tail_recording_work, K_MSEC(CONFIG_MIC_RECORDING_TAIL_MSEC));
					} else {
						mic_stop_recording(NULL);
					}
				}
			}
		}
	}

	static bool init_mic_gpio(void)
	{	
		static bool mic_gpio_init = true;
		int ret;
		if (mic_gpio_init)
		{
			if (!device_is_ready(mic_enable_gpio.port)) 
			{
				LOG_ERR("%s is not ready", mic_enable_gpio.port->name);
				return false;
			}

			ret = gpio_pin_configure_dt(&mic_enable_gpio, GPIO_OUTPUT_INACTIVE /* GPIO_OUTPUT_ACTIVE */);
			if (ret < 0) 
			{
				LOG_ERR("Failed to configure %s pin %d: %d", mic_enable_gpio.port->name, mic_enable_gpio.pin, ret);
				return false;
			}

			LOG_INF("GPIO %d configuration completed", mic_enable_gpio.pin);

			if (!device_is_ready(mic_oe_gpio.port)) 
			{
				LOG_ERR("%s is not ready", mic_oe_gpio.port->name);
				return false;
			}

			ret = gpio_pin_configure_dt(&mic_oe_gpio, GPIO_OUTPUT_INACTIVE /* GPIO_OUTPUT_ACTIVE */);
			if (ret < 0) 
			{
				LOG_ERR("Failed to configure %s pin %d: %d", mic_oe_gpio.port->name, mic_oe_gpio.pin, ret);
				return false;
			}

			LOG_INF("GPIO %d configuration completed", mic_oe_gpio.pin);

			if (!gpio_is_ready_dt(&mic_wake_gpio)) {
				LOG_ERR("%s is not ready", mic_wake_gpio.port->name);
				return -1;
			}

			int ret = gpio_pin_configure_dt(&mic_wake_gpio, GPIO_INPUT | GPIO_PULL_DOWN);
			if (ret < 0) {
				LOG_ERR("Failed to configure %s pin %d: %d", mic_wake_gpio.port->name, mic_wake_gpio.pin, ret);
				return ret;
			}

			ret = gpio_pin_interrupt_configure_dt(&mic_wake_gpio, GPIO_INT_EDGE_BOTH);
			if (ret < 0) {
				LOG_ERR("Failed to configure interrupt on %s pin %d: %d", mic_wake_gpio.port->name, mic_wake_gpio.pin, ret);
				return ret;
			}

			gpio_init_callback(&mic_wake_cb_data, mic_wake_handler, BIT(mic_wake_gpio.pin));
			gpio_add_callback(mic_wake_gpio.port, &mic_wake_cb_data);

			LOG_INF("GPIO %d configuration completed", mic_wake_gpio.pin);

			return true;
		}
	}
	
	void mic_start_recording(struct k_work* work)
	{
		ARG_UNUSED(work);
		if (gpio_pin_get_dt(&mic_wake_gpio) == 1) {
			LOG_DBG("Start recording");
			recorder_enable_record_saving();
		}		
	}
	
	K_WORK_DELAYABLE_DEFINE(mic_head_recording_work, mic_start_recording);

	void mic_stop_recording(struct k_work* work)
	{
		ARG_UNUSED(work);
		if (gpio_pin_get_dt(&mic_wake_gpio) == 0) {
			LOG_DBG("Recording STOPPED after tail period : %d ms", CONFIG_MIC_RECORDING_TAIL_MSEC);
			recorder_disable_record_saving();
		}		
	}

	K_WORK_DELAYABLE_DEFINE(mic_tail_recording_work, mic_stop_recording);

#else
	struct gpio_dt_spec mic_wake_gpio = NULL;
	struct gpio_dt_spec mic_enable_gpio = NULL;
	struct gpio_dt_spec mic_oe_gpio = NULL;
	bool init_mic_gpio(void)	{ return true; }
	void mic_wake_handler(const struct device *dev, struct gpio_callback *cb, uint32_t pins)	{ return true; }
	bool init_mic_wake_gpio(void) 	{ return true; }
#endif //#if DT_NODE_HAS_STATUS(MIC_WAKE_NODE, okay) & DT_NODE_HAS_STATUS(MIC_ENABLE_NODE, okay) & DT_NODE_HAS_STATUS(MIC_OE_NODE, okay)

// Mic config GPIO handlers
#if DT_NODE_HAS_STATUS(MIC_CLK_NODE, okay) & DT_NODE_HAS_STATUS(MIC_THSEL_NODE, okay)
	struct gpio_dt_spec mic_clk_gpio    = GPIO_DT_SPEC_GET(MIC_CLK_NODE, gpios);
	struct gpio_dt_spec mic_thsel_gpio  = GPIO_DT_SPEC_GET(MIC_THSEL_NODE, gpios);

	static bool init_mic_config_gpio(void)
	{	
		if (!gpio_is_ready_dt(&mic_clk_gpio)) {
			LOG_ERR("%s is not ready", mic_clk_gpio.port->name);
			return false;
		}

		if (!gpio_is_ready_dt(&mic_thsel_gpio)) {
			LOG_ERR("%s is not ready", mic_thsel_gpio.port->name);
			return false;
		}

		int ret = gpio_pin_configure_dt(&mic_clk_gpio, GPIO_OUTPUT);
		if (ret < 0) {
			LOG_ERR("Failed to configure %s pin %d: %d", mic_clk_gpio.port->name, mic_clk_gpio.pin, ret);
			return false;
		}

		LOG_INF("GPIO %d configuration completed", mic_clk_gpio.pin);

		ret = gpio_pin_configure_dt(&mic_thsel_gpio, GPIO_OUTPUT);
		if (ret < 0) {
			LOG_ERR("Failed to configure %s pin %d: %d", mic_thsel_gpio.port->name, mic_thsel_gpio.pin, ret);
			return false;
		}

		LOG_INF("GPIO %d configuration completed", mic_thsel_gpio.pin);

		return true;
	}

	static bool config_mic(void)
	{	
		struct t5848_config_container config;
		set_power_on_mic(true);
		enable_output_on_mic(true);
		is_mic_set = false;

		config.type = CONFIG_T5848_AAD_TYPE;

		// Config AAD A
		if (config.type == T5848_CONF_AAD_A) {
			config.config.a.aad_select = CONFIG_T5848_AAD_A_SELECT;
			config.config.a.aad_a_lpf = CONFIG_T5848_AAD_A_LPF;
			config.config.a.aad_a_thr = CONFIG_T5848_AAD_A_THR;

		// Config AAD D
		} else if(config.type == T5848_CONF_AAD_D) {
			config.config.d.aad_select = CONFIG_T5848_AAD_D_SELECT;
			config.config.d.aad_d_algo_sel = CONFIG_T5848_AAD_D_ALGO_SEL;
			config.config.d.aad_d_floor = CONFIG_T5848_AAD_D_FLOOR;
			config.config.d.aad_d_rel_pulse_min = CONFIG_T5848_AAD_D_REL_PULSE_MIN;
			config.config.d.aad_d_abs_pulse_min = CONFIG_T5848_AAD_D_ABS_PULSE_MIN;
			config.config.d.aad_d_abs_thr = CONFIG_T5848_AAD_D_ABS_THR;
			config.config.d.aad_d_rel_thr = CONFIG_T5848_AAD_D_REL_THR;
		}

		int ret = t5848_write_config(&config, &mic_clk_gpio, &mic_thsel_gpio);
		if (ret < 0) {
			LOG_ERR("Failed to configure microphone.");
			enable_output_on_mic(false);
			set_power_on_mic(false);
			return false;
		}		
		enable_output_on_mic(false);
		return true;
	}
#else
	struct gpio_dt_spec mic_clk_gpio    = NULL;
	struct gpio_dt_spec mic_thsel_gpio  = NULL;

	bool init_mic_config_gpio(void)		{ return true; }	
	bool release_mic_config_gpio(void)	{ return true; }
	bool config_mic(void)	{ return true; } 
#endif // #if DT_NODE_HAS_STATUS(MIC_CLK_NODE, okay) & DT_NODE_HAS_STATUS(MIC_THSEL_NODE, okay)

void set_power_on_sd(bool active)
{
	#if DT_NODE_HAS_STATUS(SD_ENABLE_NODE, okay)
	{
		int ret;
		if (active) {
			ret = gpio_pin_set_dt(&sd_gpio, 1);
			if (ret == 0) {
				LOG_DBG("Power on SD card is set !");
			} else {
				LOG_ERR("gpio_pin_set_dt(&sd_gpio, 1) FAILED ! Error: %d", ret);
			}
		} else {
			ret = gpio_pin_set_dt(&sd_gpio, 0);
			if (ret == 0) {
				LOG_DBG("Power on SD card is OFF !");
			} else {
				LOG_ERR("gpio_pin_set_dt(&sd_gpio, 1) FAILED ! Error: %d", ret);
			}
		}
	}
	#endif // #if DT_NODE_HAS_STATUS(SD_ENABLE_NODE, okay)	
}

void set_power_on_mic(bool active)
{
	#if DT_NODE_HAS_STATUS(MIC_ENABLE_NODE, okay)
	{
		int ret;
		if (active) {
			ret = gpio_pin_set_dt(&mic_enable_gpio, 1);
			if (ret == 0) {
				LOG_DBG("Power on mic is set !");
			} else {
				LOG_ERR("gpio_pin_set_dt(&mic_enable_gpio, 1) FAILED ! Error: %d", ret);
			}
		} else {
			ret = gpio_pin_set_dt(&mic_enable_gpio, 0);
			if (ret == 0) {
				is_mic_set = false;
				LOG_DBG("Power on mic is OFF !");
			} else {
				LOG_ERR("gpio_pin_set_dt(&mic_enable_gpio, 1) FAILED ! Error: %d", ret);
			}
		}
	}
	#endif // #if DT_NODE_HAS_STATUS(MIC_ENABLE_NODE, okay)	
}

void enable_output_on_mic(bool active)
{
	#if DT_NODE_HAS_STATUS(MIC_OE_NODE, okay)
	{
		int ret;
		if (active) {
			ret = gpio_pin_set_dt(&mic_oe_gpio, 1);
			if (ret == 0) {
				LOG_DBG("Output enable on mic !");
			} else {
				LOG_ERR("gpio_pin_set_dt(&mic_oe_gpio, 1) FAILED ! Error: %d", ret);
			}
		} else {
			ret = gpio_pin_set_dt(&mic_oe_gpio, 0);
			if (ret == 0) {
				is_mic_set = false;
				LOG_DBG("Output disable on mic !");
			} else {
				LOG_ERR("gpio_pin_set_dt(&mic_oe_gpio, 1) FAILED ! Error: %d", ret);
			}
		}
	}
	#endif // #if DT_NODE_HAS_STATUS(MIC_OE_NODE, okay)	
}

static bool handle_spi_action(bool active)
{
	#if DT_NODE_HAS_STATUS(SPI_NODE, okay)
	{
		uint8_t spi_state; 
		int ret = pm_device_state_get(spi_dev, &spi_state);
		if (ret == 0) {
			LOG_INF("SPI state: %s", pm_device_state_str(spi_state));
			if (active) {
				if (spi_state == PM_DEVICE_STATE_SUSPENDING || spi_state == PM_DEVICE_STATE_SUSPENDED) {
					gpio_hal_connect_spi_gpio();

					LOG_DBG("Resume the SPI ...");
					ret = pm_device_action_run(spi_dev, PM_DEVICE_ACTION_RESUME);
					if (ret != 0) {
						LOG_ERR("pm_device_action_run(spi_dev, PM_DEVICE_ACTION_RESUME) FAILED ! Error: %d", ret);
					}
				} else if (spi_state == PM_DEVICE_STATE_ACTIVE) {
					return true;
				}
			} else {
				if (spi_state == PM_DEVICE_STATE_ACTIVE) {
					LOG_DBG("Suspend the SPI ...");
					ret = pm_device_action_run(spi_dev, PM_DEVICE_ACTION_SUSPEND);
					if (ret != 0) {
						LOG_ERR("pm_device_action_run(spi_dev, PM_DEVICE_ACTION_SUSPEND) FAILED ! Error: %d", ret);
					}
				} else if (spi_state == PM_DEVICE_STATE_SUSPENDED) {
					gpio_hal_disconnect_spi_gpio();
					return true;
				}
			}
		} else {
			if (ret == -ENOSYS) {
				LOG_ERR("Function not implemented for SPI !");
				return true;
			} else
				LOG_ERR("pm_device_state_get(spi_dev, &spi_state) FAILED ! Error: %d", ret);
		}
		return false;
	}
	#else
		return true;
	#endif // #if DT_NODE_HAS_STATUS(SPI_NODE, okay)
}

static bool handle_i2s_action(bool active)
{
	#if DT_NODE_HAS_STATUS(I2S_NODE, okay)
	{
		uint8_t i2s_state; 
		int ret = pm_device_state_get(i2s_dev, &i2s_state);
		if (ret == 0) {
			LOG_INF("I2S state: %s", pm_device_state_str(i2s_state));
			if (active) {
				if (i2s_state == PM_DEVICE_STATE_SUSPENDED) {
					gpio_hal_connect_i2s_gpio();
					
					LOG_DBG("Resume the I2S ...");
					ret = pm_device_action_run(i2s_dev, PM_DEVICE_ACTION_RESUME);
					if (ret != 0) {
						LOG_ERR("pm_device_action_run(i2s_dev, PM_DEVICE_ACTION_RESUME) FAILED ! Error: %d", ret);
					}
				} else if (i2s_state == PM_DEVICE_STATE_ACTIVE) {
					LOG_DBG("Nothing to do !");
					return true;
				}
			} else {
				if (i2s_state == PM_DEVICE_STATE_ACTIVE) {
					LOG_DBG("Suspend the I2S ...");
					ret = pm_device_action_run(i2s_dev, PM_DEVICE_ACTION_SUSPEND);
					if (ret != 0) {
						LOG_ERR("pm_device_action_run(i2s_dev, PM_DEVICE_ACTION_SUSPEND) FAILED ! Error: %d", ret);
					}
				} else if (i2s_state == PM_DEVICE_STATE_SUSPENDED) {
					gpio_hal_disconnect_i2s_gpio();
					return true;
				}
			}
		} else {
			if (ret == -ENOSYS) {
				LOG_ERR("Function not implemented for I2S !");
				return true;
			} else
				LOG_ERR("pm_device_state_get(i2s_dev, &i2s_state) FAILED ! Error: %d", ret);
		}
		return false;
	}
	#else
		return true;
	#endif // #if DT_NODE_HAS_STATUS(I2S_NODE, okay)
}

void enable_hardware_drivers(void) 
{
	set_power_on_sd(true);
	set_power_on_mic(true);
	enable_output_on_mic(true);
	
	k_msleep(250);

	LOG_DBG("Enabling SPI ...");
	while (!handle_spi_action(true)) {
		k_msleep(500);
	}
	LOG_DBG("SPI is enabled !");

	/* PM is not supported by I2S driver */
	LOG_DBG("Enabling I2S ...");
	while (!handle_i2s_action(true)) {
		k_msleep(500);
	}
	gpio_hal_connect_i2s_gpio();	// Call because PM is not supported by I2S driver
	LOG_DBG("I2S is enabled !");

	is_sd_gpio_set = true;
	// Must wait some time to be sure that all threads have detected the power on hardware
	k_msleep(CONFIG_DELAY_ON_HARDWARE_POWERED_ON);
}

void disable_hardware_drivers(void)
{
	LOG_DBG("Disabling SPI ...");
	while (!handle_spi_action(false)) {
		k_msleep(500);
	}
	LOG_DBG("SPI is disabled !");

	/* PM is not supported by I2S driver */
	LOG_DBG("Disabling I2S ...");
	while (!handle_i2s_action(false)) {
		k_msleep(500);
	}
	gpio_hal_disconnect_i2s_gpio();	// Call because PM is not supported by I2S driver
	LOG_DBG("I2S is disabled !");

	set_power_on_sd(false);
	is_sd_gpio_set = false;
	//set_power_on_mic(false);
	//enable_output_on_mic(false);

}

void put_device_in_power_save_mode(void)
{
	// Put device in power saving mode
	LOG_WRN("Put device in power saving mode");
	k_sem_give(&low_energy_mode_sem);
}

void system_reset(void)
{
	LOG_WRN("NVIC_SystemReset() will be called soon !");

	// Stop any BLE activity on Network core
	LOG_DBG("Stopping any BLE activity ...");
	ble_thread_running = false;

	// Checking BLE activity ...
	k_sem_take(&thread_ble_busy_sem, K_FOREVER);
	LOG_INF("No more BLE activity running");

	// Waiting for end of fat fs thread
	k_sem_take(&thread_fatfs_busy_sem, K_FOREVER);

	// Update Hot Reset Flag
	hot_reset = CONFIG_HOT_RESET_VAL;
	// k_msleep(2000);

	// Applying the Software Reset
	NVIC_SystemReset();
}

const char* main_state_to_string(void)
{
	switch (main_state)
	{
	case ST_INIT:			return "ST_IDLE";
	case ST_WAIT_SD_CARD:	return "ST_WAIT_SD_CARD";
	case ST_IDLE:			return "ST_IDLE";
	case ST_RECORDING:		return "ST_RECORDING";
	case ST_DISK_FULL:		return "ST_DISK_FULL";
	case ST_LOW_BATT:		return "ST_LOW_BATT";
	case ST_POWER_SAVING:	return "ST_POWER_SAVING";
	case ST_ERROR:			return "ST_ERROR";
	default:				return "Unknown state!";
	}
}


static void main_thread(void) 
{
	// Taking semaphores of all other threads
	k_sem_take(&thread_i2s_busy_sem, K_NO_WAIT);
	k_sem_take(&thread_store_busy_sem, K_NO_WAIT);
	k_sem_take(&thread_ble_busy_sem, K_NO_WAIT);
	k_sem_take(&thread_fatfs_busy_sem, K_NO_WAIT);

	// Initialize global variables that MUST be initialized with a HOT reset
	is_sd_gpio_set 				= false;
	is_collar_burn_gpio_set 	= false;
	is_low_batt_detected 		= false;
	is_main_thread_initialized 	= false;

	gpio_hal_disconnect_all_unused();

	if (hot_reset != CONFIG_HOT_RESET_VAL) {
		// Initialize NON Init variables
		must_be_in_power_saving_mode = false;
		main_state = ST_INIT;
	} else {
		#if defined(CONFIG_BOARD_NRF5340DK)
		{
			// errata - Tested this workaround. Be aware of the register address:
			// - 0x50005000 for secure firmware
			// - 0x40005000 for non-secure firmware
			*(volatile uint32_t *) 0x40005618ul = 1ul;
			NRF_RESET->NETWORK.FORCEOFF = (RESET_NETWORK_FORCEOFF_FORCEOFF_Release << RESET_NETWORK_FORCEOFF_FORCEOFF_Pos);
			k_msleep(5); // Wait for at least five microseconds
			NRF_RESET->NETWORK.FORCEOFF = (RESET_NETWORK_FORCEOFF_FORCEOFF_Hold << RESET_NETWORK_FORCEOFF_FORCEOFF_Pos);
			k_msleep(1); // Wait for at least one microsecond
			NRF_RESET->NETWORK.FORCEOFF = (RESET_NETWORK_FORCEOFF_FORCEOFF_Release << RESET_NETWORK_FORCEOFF_FORCEOFF_Pos);
			*(volatile uint32_t *) 0x40005618ul = 0ul;
		}
			#endif // #if defined(CONFIG_BOARD_NRF5340DK)

	}

	// ASCII Art Generator: http://patorjk.com/software/taag
	// Font: Big
	LOG_INF("___________________________________________________________");
	LOG_INF("   _____                  _    _   _       ______     _ _ ");
	LOG_INF("  / ____|                | |  | \\ | |     |  ____|   (_) |");
	LOG_INF(" | (___  _ __   ___  __ _| | _|  \\| | ___ | |____   ___| |");
	LOG_INF("  \\___ \\| '_ \\ / _ \\/ _` | |/ / . ` |/ _ \\|  __\\ \\ / / | |");
	LOG_INF("  ____) | |_) |  __/ (_| |   <| |\\  | (_) | |___\\ V /| | |");
	LOG_INF(" |_____/| .__/ \\___|\\__,_|_|\\_\\_| \\_|\\___/|______\\_/ |_|_|");
	LOG_INF("        | |");
	LOG_INF("        |_|");
	LOG_INF("___________________________________________________________");
	LOG_INF("");
	LOG_INF("Software version: %s", FIRMWARE_REVISION);
	#ifdef CONFIG_COMPILE_FOR_MONKEY_PCB
		LOG_INF("Hardware: Monkey PCB");
	#else
		LOG_INF("Hardware: PCA10156");
	#endif
	LOG_INF("___________________________________________________________");
	LOG_DBG("Starting Speak No Evil App ... (%s)", CONFIG_BOARD);

	// Initializing FLASH
	flash_init();

	if (hot_reset != CONFIG_HOT_RESET_VAL) {
		flash_device_identifier = flash_get_device_identifier();
		ble_update_device_id_char_val();
		LOG_INF("flash_device_identifier: %d", flash_device_identifier);
		flash_mic_input_gain    = flash_get_mic_input_gain();
		ble_update_mic_gain_char_val();
		LOG_INF("flash_mic_input_gain:    %d", flash_mic_input_gain);
	}

	#if DT_NODE_HAS_STATUS(LOW_BATT_NODE, okay)
	{
		if (!init_low_batt_gpio()) {
			LOG_ERR("init_low_batt_gpio() FAILED !");
			return;
		}
	}
	#else
		k_sem_take(&low_energy_mode_sem, K_FOREVER);
	#endif // #if DT_NODE_HAS_STATUS(LOW_BATT_NODE, okay)

	#if DT_NODE_HAS_STATUS(SD_ENABLE_NODE, okay)
	{
		if (!init_sd_gpio()) {
			LOG_ERR("init_sd_mic_gpio() FAILED !");
			return;
		}
	}
	#endif // #if DT_NODE_HAS_STATUS(SD_MIC_ENABLE_NODE, okay)

	#if DT_NODE_HAS_STATUS(BURN_COLLAR_NODE, okay)
	{
		if (!init_burn_collar_gpio()) {
			LOG_ERR("init_burn_collar_gpio() FAILED !");
			return;
		}
	}
	#endif // #if DT_NODE_HAS_STATUS(BURN_COLLAR_NODE, okay)

	#if DT_NODE_HAS_STATUS(MIC_WAKE_NODE, okay) & DT_NODE_HAS_STATUS(MIC_ENABLE_NODE, okay)
	{
		if (!init_mic_gpio()) {
			LOG_ERR("init_mic_gpio() FAILED !");
			return;
		}
	}
	#endif //DT_NODE_HAS_STATUS(MIC_WAKE_NODE, okay)

	#if DT_NODE_HAS_STATUS(MIC_CLK_NODE, okay) & DT_NODE_HAS_STATUS(MIC_THSEL_NODE, okay)
	{
		if (!init_mic_config_gpio()) {
			LOG_ERR("init_mic_conf_gpio() FAILED !");
			return;
		}

		if (!config_mic()) {
			LOG_ERR("config_mic() failed to start!");
			return;
		}

		int ret = k_sem_take(&mic_config_done_sem, K_MSEC(10));

		if (ret != 0) {
			LOG_ERR("Microphone configuration confirmation timeout! Error: %d", ret);
			LOG_ERR("Microphone configuration failed !");
			return;
		}

		gpio_hal_disconnect_mic_config_gpio();

	}
	#endif //DT_NODE_HAS_STATUS(MIC_CLK_NODE, okay) & DT_NODE_HAS_STATUS(MIC_THSEL_NODE, okay)

	struct tm tm_;
	if (hot_reset != CONFIG_HOT_RESET_VAL) {
		total_days_of_records = 0;
		LOG_DBG("FIRST initialization of `total_days_of_records` to %d", total_days_of_records);
		file_idx = 1;
		LOG_DBG("FIRST initialization of `file_idx` to %d", file_idx);
		clock_gettime(CLOCK_REALTIME, &start_time_ts);
		time(&start_time_ts.tv_sec);
		localtime_r(&start_time_ts.tv_sec, &tm_);
		start_day = tm_.tm_mday;
		start_month = tm_.tm_mon;
		LOG_DBG("FIRST initialization of `start_day` and `start_month` to %d and %d", start_day, start_month);
	} else {
		LOG_WRN("+++++++++++ Hot Reset detected +++++++++++");
		clock_settime(CLOCK_REALTIME, &hot_reset_ts);
		LOG_DBG("Main State: %s", main_state_to_string());
	}

	// Giving semaphores of all other threads
	k_sem_give(&thread_i2s_busy_sem);
	k_sem_give(&thread_store_busy_sem);
	k_sem_give(&thread_ble_busy_sem);
	k_sem_give(&thread_fatfs_busy_sem);

	while (!must_be_in_power_saving_mode) {
		k_sem_take(&low_energy_mode_sem, K_FOREVER);
		LOG_WRN("Handling Power Saving Mode ! ...");
		must_be_in_power_saving_mode = true;
		ble_update_status_and_dor(main_state, total_days_of_records);

		// Workaround to disable completely FAT_fs
		if (is_sd_gpio_set) {
			// Stop recording
			LOG_DBG("Stop recording ...");
			recorder_disable_record();

			system_reset();
			// k_msleep(3000);
			// hot_reset = CONFIG_HOT_RESET_VAL;
			// NVIC_SystemReset();
		}
	}

	// Around GPIOs
	// Will be done only if Low Batt detected -> moved in low_batt_debounced() method
	// gpio_hal_disconnect_low_batt_gpio();

	// No more needed, it's done in `disable_hardware_drivers()` method !
	// gpio_hal_disconnect_i2s_gpio();
	// gpio_hal_disconnect_spi_gpio();
	LOG_WRN("------------ Main Thread ended ... ------------\n");
}

// Define and initialize the threads
K_THREAD_DEFINE(thread_main_id,  THREAD_MAIN_STACKSIZE,  main_thread, NULL, NULL, NULL, THREAD_MAIN_PRIORITY, 0, 0);
K_THREAD_DEFINE(thread_fatfs_id, THREAD_FATFS_STACKSIZE, sdcard_thread_fatfs_mount, &main_mp, NULL, NULL, THREAD_FATFS_PRIORITY, 0, 0);
K_THREAD_DEFINE(thread_audio_id, THREAD_AUDIO_STACKSIZE, recorder_thread_i2s, NULL, NULL, NULL, THREAD_AUDIO_PRIORITY, 0, 0);
K_THREAD_DEFINE(thread_file_id,  THREAD_FILE_STACKSIZE,  recorder_thread_store_to_file, NULL, NULL, NULL, THREAD_FILE_PRIORITY, 0, 0);
K_THREAD_DEFINE(thread_ble_id,   THREAD_BLE_STACKSIZE,   ble_thread_init, NULL, NULL, NULL, THREAD_BLE_PRIORITY, 0, 0);
