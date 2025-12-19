/*
 * Copyright (c) 2023, HES-SO Valais-Wallis, HEI, Sion
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "recorder.h"

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/fs/fs.h>
#include <zephyr/logging/log.h>
#include <zephyr/logging/log.h>
#include <zephyr/posix/time.h>

#include <errno.h>

#include <stdlib.h>
#include <time.h>

#include "../fatfs/sdcard.h"
#include "../ble/ble_handler.h"
#include "../define.h"

LOG_MODULE_REGISTER(recorder, CONFIG_RECORDER_LOG_LEVEL);

#if DT_NODE_EXISTS(DT_NODELABEL(i2s_rxtx))
	#define I2S_RX_NODE  	DT_NODELABEL(i2s_rxtx)
	#define I2S_TX_NODE  	I2S_RX_NODE
#else
	#if DT_NODE_EXISTS(DT_NODELABEL(i2s_rx))
		#define I2S_RX_NODE DT_NODELABEL(i2s_rx)
	#else
		#define I2S_RX_NODE NULL
	#endif

	#if DT_NODE_EXISTS(DT_NODELABEL(i2s_tx))
		#define I2S_TX_NODE  DT_NODELABEL(i2s_tx)
	#else
		#define I2S_TX_NODE  NULL
	#endif
#endif

#ifndef CONFIG_COMPILE_FOR_MONKEY_PCB
	#define SW0_NODE        DT_ALIAS(sw0)
#endif
#define I2S_SEL_NODE		DT_ALIAS(sel0)


#define I2S_SAMPLES_PER_BLOCK           ((CONFIG_I2S_SAMPLE_FREQUENCY / CONFIG_I2S_SAMPLE_FREQUENCY_DIVIDER) * CONFIG_I2S_NUMBER_OF_CHANNELS)
#define I2S_BLOCK_SIZE                  (CONFIG_I2S_BYTES_PER_SAMPLE * I2S_SAMPLES_PER_BLOCK)
#define I2S_BLOCK_COUNT                 (CONFIG_I2S_INITIAL_BLOCKS + 2)
#ifdef CONFIG_BOARD_NRF52840DK_NRF52840
	#define I2S_BUFFER_SIZE_FACTOR		1
#endif

#ifdef CONFIG_BOARD_NRF5340DK_NRF5340_CPUAPP_NS
	#define I2S_BUFFER_SIZE_FACTOR		2
#endif

#ifdef CONFIG_BOARD_NRF5340DK_NRF5340_CPUAPP
	#define I2S_BUFFER_SIZE_FACTOR		2
#endif

#ifdef CONFIG_BOARD_NRF54L15DK_NRF54L15_CPUAPP_NS
	#define I2S_BUFFER_SIZE_FACTOR		1
#endif

#ifdef CONFIG_BOARD_NRF54L15DK_NRF54L15_CPUAPP
	#define I2S_BUFFER_SIZE_FACTOR		1
#endif

#ifndef I2S_BUFFER_SIZE_FACTOR
	#error "I2S_BUFFER_SIZE_FACTOR not defined"asm
#endif

static bool is_recorder_enable;
static bool recorder_store_flag;
static bool is_file_opened;
static bool file_needs_init;
bool is_saving_enable;
static int32_t recorder_sample_offset;
static uint8_t recorder_store_idx;

K_MEM_SLAB_DEFINE_STATIC(mem_slab, I2S_BLOCK_SIZE, I2S_BLOCK_COUNT, 4);

static const uint8_t 	r_shift 				= (CONFIG_I2S_SAMPLE_BIT_WIDTH - CONFIG_STORAGE_SAMPLE_BIT_WIDTH)/*+2*/;
static const uint32_t 	store_block_size		= (CONFIG_I2S_SAMPLE_FREQUENCY_DIVIDER * I2S_SAMPLES_PER_BLOCK * CONFIG_STORAGE_BYTES_PER_SAMPLE * I2S_BUFFER_SIZE_FACTOR);
static int8_t 			store_block[2][(CONFIG_I2S_SAMPLE_FREQUENCY_DIVIDER * I2S_SAMPLES_PER_BLOCK * CONFIG_STORAGE_BYTES_PER_SAMPLE * I2S_BUFFER_SIZE_FACTOR) + 1];

static struct tm tm_;
static time_t now;

// At start up, the recording is disabled
K_SEM_DEFINE(recorder_toggle_transfer_sem, 1, 1);
K_SEM_DEFINE(recorder_toggle_saving_sem, 0, 1);
K_SEM_DEFINE(thread_i2s_busy_sem, 1, 1);
K_SEM_DEFINE(thread_store_busy_sem, 1, 1);

#ifndef CONFIG_COMPILE_FOR_MONKEY_PCB
	#if DT_NODE_HAS_STATUS(SW0_NODE, okay)
		static struct gpio_dt_spec sw0_spec 	= GPIO_DT_SPEC_GET(SW0_NODE, gpios);
		static inline void sw0_handler(const struct device *dev, struct gpio_callback *cb, uint32_t pins) { 
			k_sem_give(&recorder_toggle_transfer_sem);
		}
	#endif
#endif // #ifndef CONFIG_COMPILE_FOR_MONKEY_PCB

#if DT_NODE_HAS_STATUS(I2S_SEL_NODE, okay)
	static struct gpio_dt_spec i2s_sel_spec 	= GPIO_DT_SPEC_GET(I2S_SEL_NODE, gpios);
#endif // #if DT_NODE_HAS_STATUS(I2S_SEL_NODE, okay)

struct fs_file_t file;

static const char* recorder_trigger_cmd_to_string(enum i2s_trigger_cmd cmd)
{
	switch (cmd) {
		case I2S_TRIGGER_START:		return "I2S_TRIGGER_START";
		case I2S_TRIGGER_STOP:		return "I2S_TRIGGER_STOP";
		case I2S_TRIGGER_DRAIN:		return "I2S_TRIGGER_DRAIN";
		case I2S_TRIGGER_DROP:		return "I2S_TRIGGER_DROP";
		case I2S_TRIGGER_PREPARE:	return "I2S_TRIGGER_PREPARE";
		default : return "Uknown cmd";
	}
}

static inline void recorder_incr_store_idx(void)		{ recorder_store_idx = (recorder_store_idx + 1) & 0x01; }
static inline uint8_t recorder_store_idx_to_write(void)	{ return ((recorder_store_idx - 1) & 0x01); }

static bool recorder_init_buttons(void)
{
	#ifndef CONFIG_COMPILE_FOR_MONKEY_PCB
	{
		#if DT_NODE_HAS_STATUS(SW0_NODE, okay)
		{
			int ret;
			static struct gpio_callback sw0_cb_data;

			if (!device_is_ready(sw0_spec.port)) {
				LOG_ERR("%s is not ready (sw0_spec)", sw0_spec.port->name);
				return false;
			}

			ret = gpio_pin_configure_dt(&sw0_spec, GPIO_INPUT);
			if (ret < 0) {
				LOG_ERR("Failed to configure %s pin %d: %d", sw0_spec.port->name, sw0_spec.pin, ret);
				return false;
			}

			ret = gpio_pin_interrupt_configure_dt(&sw0_spec, GPIO_INT_EDGE_TO_ACTIVE);
			if (ret < 0) {
				LOG_ERR("Failed to configure interrupt on %s pin %d: %d", sw0_spec.port->name, sw0_spec.pin, ret);
				return false;
			}

			gpio_init_callback(&sw0_cb_data, sw0_handler, BIT(sw0_spec.pin));
			gpio_add_callback(sw0_spec.port, &sw0_cb_data);
			LOG_INF("Press Button 1 to stop/restart I2S streams\n");
		}
		#endif // #if DT_NODE_HAS_STATUS(SW0_NODE, okay)
	}
	#endif // #ifndef CONFIG_COMPILE_FOR_MONKEY_PCB

	#if DT_NODE_HAS_STATUS(I2S_SEL_NODE, okay)
	{
		int ret;
		if (!device_is_ready(i2s_sel_spec.port)) {
			LOG_ERR("%s is not ready (i2s_sel_spec)", i2s_sel_spec.port->name);
			return false;
		}

		ret = gpio_pin_configure_dt(&i2s_sel_spec, GPIO_OUTPUT_INACTIVE);
		if (ret < 0) {
			LOG_ERR("Failed to configure %s pin %d: %d", i2s_sel_spec.port->name, i2s_sel_spec.pin, ret);
			return false;
		}
	}
	#endif // #if DT_NODE_HAS_STATUS(I2S_SEL_NODE, okay)

	return true;
}

void recorder_enable_record(void)
{
	LOG_WRN("recorder_toggle_transfer_sem count: %d, is_recorder_enable ? %s", k_sem_count_get(&recorder_toggle_transfer_sem), is_recorder_enable ? "YES":"NO");
	if (!is_recorder_enable) {
		k_sem_give(&recorder_toggle_transfer_sem);
	}
}
void recorder_disable_record(void)
{
	LOG_WRN("recorder_toggle_transfer_sem count: %d, is_recorder_enable ? %s", k_sem_count_get(&recorder_toggle_transfer_sem), is_recorder_enable ? "YES":"NO");
	if (is_recorder_enable) {
		LOG_DBG("Should stop recording...");
		k_sem_give(&recorder_toggle_transfer_sem);
	}
}

void recorder_enable_record_saving(void)
{
	LOG_WRN("recorder_toggle_saving_sem count: %d, is_recorder_enable ? %s", k_sem_count_get(&recorder_toggle_saving_sem), is_saving_enable ? "YES":"NO");
	if (!is_saving_enable) {
		k_sem_give(&recorder_toggle_saving_sem);
	}
}

void recorder_disable_record_saving(void)
{
	LOG_WRN("recorder_toggle_saving_sem count: %d, is_recorder_enable ? %s", k_sem_count_get(&recorder_toggle_saving_sem), is_saving_enable ? "YES":"NO");
	if (is_saving_enable) {
		LOG_DBG("Should stop recording...");
		k_sem_give(&recorder_toggle_saving_sem);
	}
}

void recorder_i2s_initialize(struct i2s_config *config)
{
	LOG_INF("Initializing I2S interface...");
	config->word_size = CONFIG_I2S_SAMPLE_BIT_WIDTH;
	config->channels = CONFIG_I2S_NUMBER_OF_CHANNELS;
	config->format = I2S_FMT_DATA_FORMAT_I2S;
	config->options = I2S_OPT_BIT_CLK_MASTER | I2S_OPT_FRAME_CLK_MASTER;
	config->frame_clk_freq = CONFIG_I2S_SAMPLE_FREQUENCY;
	config->mem_slab = &mem_slab;
	config->block_size = I2S_BLOCK_SIZE;
	config->timeout = CONFIG_I2S_TIMEOUT;
}

void recorder_get_dc_offset(void* samples, uint32_t samples_size)
{
	int64_t dc_sum = 0;
    int32_t *ptr = (int32_t *)samples;

    for (int i = 0; i < samples_size; i++) {
        int16_t sample_16bit = (int16_t)(ptr[i] >> 8);
        dc_sum += sample_16bit;
    }
    recorder_sample_offset = (int32_t)(dc_sum / samples_size);
    LOG_DBG("DC Offset: %d\n", recorder_sample_offset);
}

int32_t recorder_normalize_sample(int32_t sample, int gain, int divider, uint8_t rshift)
{
	int32_t val = (int32_t)((int16_t)(sample >> rshift));

    int64_t s = (int64_t)val - recorder_sample_offset;

    if (divider > 1) {
        s = (s * gain) / divider;
    } else if (gain > 1) {
        s = s * gain;
    }

    return (int32_t)s;
}

bool recorder_configure_streams(const struct device *i2s_dev_rx, const struct device *i2s_dev_tx, const struct i2s_config *config)
{
	int ret;

	if (i2s_dev_rx == i2s_dev_tx) {
		LOG_DBG("I2S_DIR_BOTH ...");
		ret = i2s_configure(i2s_dev_rx, I2S_DIR_BOTH, config);
		if (ret == 0) {
			LOG_DBG("COMPLETED");
			return true;
		}
		/* 
		 * -ENOSYS means that the RX and TX streams need to be configured separately.
		 */
		if (ret != -ENOSYS) {
			LOG_ERR("Failed to configure streams: %d\n", ret);
			return false;
		}
	}

	if (i2s_dev_rx != NULL) {
		LOG_DBG("I2S_DIR_RX ...");
		ret = i2s_configure(i2s_dev_rx, I2S_DIR_RX, config);
		if (ret < 0) {
			LOG_ERR("Failed to configure RX stream: %d\n", ret);
			return false;
		}
		LOG_DBG("COMPLETED");
	}

	if (i2s_dev_tx != NULL) {
		LOG_DBG("I2S_DIR_TX ...");
		ret = i2s_configure(i2s_dev_tx, I2S_DIR_TX, config);
		if (ret < 0) {
			LOG_ERR("Failed to configure TX stream: %d\n", ret);
			return false;
		}
		LOG_DBG("COMPLETED");

	}
	return true;
}

bool recorder_prepare_transfer(const struct device *i2s_dev_rx, const struct device *i2s_dev_tx)
{
	int ret;

	for (int i = 0; i < CONFIG_I2S_INITIAL_BLOCKS; ++i) {
		void *mem_block;

		ret = k_mem_slab_alloc(&mem_slab, &mem_block, K_NO_WAIT);
		if (ret < 0) {
			LOG_ERR("Failed to allocate block %d: %d\n", i, ret);
			return false;
		}

		memset(mem_block, 0, I2S_BLOCK_SIZE);

		if (i2s_dev_tx != NULL) {
			ret = i2s_write(i2s_dev_tx, mem_block, I2S_BLOCK_SIZE);
			if (ret < 0) {
				LOG_ERR("Failed to write block %d: %d\n", i, ret);
				return false;
			}
		}
	}
	return true;
}

bool recorder_trigger_command(const struct device *i2s_dev_rx, const struct device *i2s_dev_tx, enum i2s_trigger_cmd cmd)
{
	int ret;

	if (i2s_dev_rx == i2s_dev_tx) {
		LOG_DBG("I2S_DIR_BOTH, %s ...", recorder_trigger_cmd_to_string(cmd));
		ret = i2s_trigger(i2s_dev_rx, I2S_DIR_BOTH, cmd);
		if (ret == 0) {
			LOG_DBG("i2s_trigger(...) COMPLETED");
			return true;
		}
		/* 
		 * -ENOSYS means that commands for the RX and TX streams need to be triggered separately.
		 */
		if (ret != -ENOSYS) {
			LOG_ERR("Failed to trigger command %d: %d\n", cmd, ret);
			return false;
		}
	}

	if (i2s_dev_rx != NULL) {
		LOG_DBG("I2S_DIR_RX,  %s ...", recorder_trigger_cmd_to_string(cmd));
		ret = i2s_trigger(i2s_dev_rx, I2S_DIR_RX, cmd);
		if (ret < 0) {
			LOG_ERR("Failed to trigger command %d on RX: %d\n", cmd, ret);
			return false;
		}
		LOG_DBG("i2s_trigger(...) COMPLETED");
	}

	if (i2s_dev_tx != NULL) {
		LOG_DBG("I2S_DIR_TX, %s ...", recorder_trigger_cmd_to_string(cmd));
		ret = i2s_trigger(i2s_dev_tx, I2S_DIR_TX, cmd);
		if (ret < 0) {
			LOG_ERR("Failed to trigger command %d on TX: %d\n", cmd, ret);
			return false;
		}
		LOG_DBG("i2s_trigger(...) COMPLETED");
	}
	return true;
}

bool recorder_calibration(const struct device *const i2s_dev_rx, const struct device *const i2s_dev_tx) {
	void *mem_block;
	uint32_t block_size;
	int ret;
	bool is_config_done = false;
	bool config_flag = false;
	uint8_t config_flag_counter = 0;

	if (!recorder_prepare_transfer(i2s_dev_rx, i2s_dev_tx)) {
		LOG_ERR("prepare_transfer FAILED");
		return false;
	}

	if (!recorder_trigger_command(i2s_dev_rx, i2s_dev_tx, I2S_TRIGGER_START)) {
		LOG_ERR("trigger_command I2S_TRIGGER_START failed");
		return false;
	}

	ret = i2s_read(i2s_dev_rx, &mem_block, &block_size);
	
	recorder_get_dc_offset(mem_block, I2S_SAMPLES_PER_BLOCK);

	if (i2s_dev_tx != NULL) {
		i2s_write(i2s_dev_tx, mem_block, block_size);
	} else {
		k_mem_slab_free(&mem_slab, &mem_block);
	}

	if (!recorder_trigger_command(i2s_dev_rx, i2s_dev_tx, I2S_TRIGGER_DROP)) {
		LOG_ERR("trigger_command I2S_TRIGGER_DROP failed");
		return false;
	}

	gpio_hal_force_low_i2s_gpio();

	return true;
}

// Around Thread
void recorder_thread_i2s(void) 
{
	// Initialization of the RAM in case of Hot Reset (software reset)
	is_recorder_enable		= false;
	recorder_store_flag		= false;
	is_file_opened			= false;
	file_needs_init			= false;

	recorder_sample_offset	= 0;
	#if DT_NODE_HAS_STATUS(I2S_NODE, okay)
	{
		const struct device *const i2s_dev_rx = DEVICE_DT_GET(I2S_RX_NODE);
		const struct device *const i2s_dev_tx = DEVICE_DT_GET(I2S_TX_NODE);
		struct i2s_config i2s_config;

		// Checking if thread could start
		k_sem_take(&thread_i2s_busy_sem, K_FOREVER);

		LOG_INF("Audio Thread for MONKEY application started ...\n");
		if (!recorder_init_buttons()) {
			return;
		}

		// Waiting until Power on I2S is set and the sdcard is ready
		while (!is_sd_gpio_set || !sdcard_is_ready()) {
			if (is_low_batt_detected || must_be_in_power_saving_mode) {
				ble_update_status_and_dor(main_state, total_days_of_records);
				LOG_WRN("------------ Audio Thread for MONKEY ended ! ------------");
				return;
			}
			k_msleep(2000);
		}
		LOG_INF("Power on I2S interface is set ...");

		if (!device_is_ready(i2s_dev_rx)) {
			LOG_ERR("%s is not ready\n", i2s_dev_rx->name);
			return;
		}

		if (i2s_dev_rx != i2s_dev_tx && !device_is_ready(i2s_dev_tx)) {
			LOG_ERR("%s is not ready\n", i2s_dev_tx->name);
			return;
		}

		LOG_INF("Initializing store_block ...");
		memset(store_block, 0x00, sizeof(store_block));
		LOG_DBG("done !\n");

		LOG_DBG("Initializing file ...");
		sdcard_file_init(&file);

		#ifndef CONFIG_I2S_RECORD_AUTO_START
		{
			if (k_sem_count_get(&recorder_toggle_transfer_sem) != 0) {
				LOG_DBG("Disabling recording ...");
				k_sem_take(&recorder_toggle_transfer_sem, K_NO_WAIT);
				is_recorder_enable = false;
			}
		}
		#endif // CONFIG_I2S_RECORD_AUTO_START

		LOG_INF("Starting I2S interface...");
		recorder_i2s_initialize(&i2s_config);
		if (!recorder_configure_streams(i2s_dev_rx, i2s_dev_tx, &i2s_config)) {
			LOG_ERR("configure_streams FAILED !\n");
			return;
		}

		int32_t* forFatPtr;
		bool calibration = true;
		uint8_t store_flag_counter;
		while (!must_be_in_power_saving_mode)
		{
			is_recorder_enable = false;
		
			if (sdcard_is_ready()) 
			{
				ble_update_status_and_dor(ST_IDLE, total_days_of_records);
				recorder_store_flag = false;
				store_flag_counter = 0;
				int sem_take = k_sem_take(&recorder_toggle_transfer_sem, K_FOREVER);
				LOG_DBG("k_sem_take(&recorder_toggle_transfer_sem, K_FOREVER): %d", sem_take);

				is_recorder_enable = true;
				ble_update_status_and_dor(ST_RECORDING, total_days_of_records);
				forFatPtr = (int32_t*) store_block[recorder_store_idx];
				
				// The recording is starting here
				sem_take = k_sem_take(&file_access_sem, K_NO_WAIT);
				LOG_DBG("k_sem_take(&file_access_sem, ...) > %d (count: %d)", sem_take, file_access_sem.count);
				while (k_sem_take(&recorder_toggle_transfer_sem, K_NO_WAIT) != 0) {
					recorder_store_idx = 0;

					// Calibration of the DC Offset
					if (calibration) {
						recorder_calibration(i2s_dev_rx, i2s_dev_tx);
						calibration = false;
					}

					if (!is_file_opened){
						file_needs_init = true;
					}

					sem_take = k_sem_take(&recorder_toggle_saving_sem, K_FOREVER);
					LOG_DBG("k_sem_take(&recorder_toggle_saving_sem, K_FOREVER): %d", sem_take);
					is_saving_enable = true;

					if (!recorder_prepare_transfer(i2s_dev_rx, i2s_dev_tx)) {
						LOG_ERR("prepare_transfer FAILED");
						return;
					}

					if (!recorder_trigger_command(i2s_dev_rx, i2s_dev_tx, I2S_TRIGGER_START)) {
						LOG_ERR("trigger_command I2S_TRIGGER_START failed");
						return;
					}

					// The wake pin is HIGH, the program start to read data from I2S and store it to the SD Card
					while (k_sem_take(&recorder_toggle_saving_sem, K_NO_WAIT) != 0) {
						void *mem_block;
						uint32_t block_size;
						int ret;

						ret = i2s_read(i2s_dev_rx, &mem_block, &block_size);
						if (ret == 0) {
							// downsampling from 24 bits to 16 bits samples
							// --------------------------------------------
							// The MEMS mic (I2S) is a 24bit microphone with 16 significant bits.
							// Bit Mapping (32-bit container):
 							// [31 ... 24] |  [23 ......... 8]  | [7 .... 0]
							//    Dummy    |     Audio Data     |    Dummy
							// To reach this goal, we simply shift the sample of 8bits (24->16)
							uint8_t right_shift = r_shift;
							for (int i = 0; i < I2S_SAMPLES_PER_BLOCK; i++)
							{
								// *forFatPtr = recorder_normalize_sample((((int32_t *) mem_block)[i]), CONFIG_I2S_MIC_INPUT_GAIN, CONFIG_I2S_MIC_INPUT_DIVIDER, 0xffffc000, right_shift);
								*forFatPtr = recorder_normalize_sample((((int32_t *) mem_block)[i]), (int) flash_mic_input_gain, CONFIG_I2S_MIC_INPUT_DIVIDER, right_shift);
								forFatPtr  = ((int8_t*) forFatPtr) + CONFIG_STORAGE_BYTES_PER_SAMPLE;
							}
						} else {
							LOG_ERR("Failed to read data: %d", ret);
							break;
						} 

						if (i2s_dev_tx != NULL)
						{
							ret = i2s_write(i2s_dev_tx, mem_block, block_size);
							if (ret < 0)
							{
								LOG_ERR("Failed to write data: %d", ret);
								break;
							}
						}

						// Trigger to store samples to the SD Card
						if (((uint32_t)forFatPtr - (uint32_t) store_block[recorder_store_idx]) >= store_block_size)
						{
							recorder_incr_store_idx();
							forFatPtr = (int32_t*) store_block[recorder_store_idx];
							k_sem_give(&file_access_sem);
						}

					}

					if (is_file_opened) {
						// Ensure that all data are sent to the SD Card before stopping saving
						recorder_incr_store_idx();
						forFatPtr = (int32_t*) store_block[recorder_store_idx];
						k_sem_give(&file_access_sem);

						file_needs_init = true;
					}

					is_saving_enable = false;

					if (!recorder_trigger_command(i2s_dev_rx, i2s_dev_tx, I2S_TRIGGER_DROP)) {
						LOG_ERR("trigger_command I2S_TRIGGER_DROP failed");
						return;
					}

					gpio_hal_force_low_i2s_gpio();
				}

				if (!recorder_trigger_command(i2s_dev_rx, i2s_dev_tx, I2S_TRIGGER_DROP)) {
					LOG_ERR("trigger_command I2S_TRIGGER_DROP failed");
					return;
				}

				gpio_hal_force_low_i2s_gpio();

				LOG_DBG("Streams stopped");
				ble_update_status_and_dor(ST_IDLE, total_days_of_records);

				sdcard_file_close(&file);
				is_file_opened = false;

				// To go back in Hardware off, we must apply a software reset !
				if (!ble_open_collar_cmd_received) {
					sdcard_off_and_reset(&main_mp, false);
				}
			}
			else
			{
				LOG_WRN("SD Card is NOT Ready!");
				time(&now);
				localtime_r(&now, &tm_);
				LOG_INF("Now: %d.%d.%d %02d:%02d:%02d", tm_.tm_mday, tm_.tm_mon+1, tm_.tm_year+1900, tm_.tm_hour, tm_.tm_min, tm_.tm_sec);
				ble_update_status_and_dor(ST_WAIT_SD_CARD, total_days_of_records);
				k_msleep(1000);
			}
		}

		ble_update_status_and_dor(main_state, total_days_of_records);
		LOG_INF("No more recording !");

				if (is_file_opened) {
			sdcard_file_close(&file);
			is_file_opened = false;
		}

		// Giving start's semaphore if thread could start
		k_sem_give(&thread_i2s_busy_sem);
		LOG_WRN("------------ Audio Thread for MONKEY ended ! ------------");
	}
	#endif // #if DT_NODE_HAS_STATUS(I2S_NODE, okay)
}

void recorder_thread_store_to_file(void) 
{
	int w_res;
	time_t end_t;
	double diff_t;

	// Checking if thread could start
	k_sem_take(&thread_store_busy_sem, K_FOREVER);

	LOG_INF("Store to File Thread for MONKEY application started ...\n");
	while (!must_be_in_power_saving_mode) 
	{
		// Init the file if needed
		if (file_needs_init) {
			// Close previous file if opened (use when changing file index)
			if (is_file_opened) {
				sdcard_file_close(&file);
				is_file_opened = false;
			}
			
			LOG_WRN("Will open file with index %d", file_idx);
			is_file_opened = sdcard_file_setup_and_open(&file, file_idx++);
			file_needs_init = false;
		} else if(is_file_opened) {
			k_sem_take(&file_access_sem, K_FOREVER);

			// Check the Low Batt mode
			if (must_be_in_power_saving_mode) {
				LOG_DBG("Device in power saving mode !");
			} else {

				time(&now);
				localtime_r(&now, &tm_);

				uint8_t idx = recorder_store_idx_to_write();
				LOG_DBG("Will store to sd card using idx: %d... (recorder_store_idx: %d)", idx, recorder_store_idx);
				time(&end_t);

				w_res = sdcard_write(&file, store_block[idx], store_block_size);
				LOG_DBG("sdcard_write(...) -> %d, store_block_size: %d ", w_res, store_block_size);

				if (w_res == -ENOMEM) {
					// Not enough space on disk => close file and stop recording
					LOG_WRN("SD Card is Full! -> closing file and going to POWER SAVE mode...");
					is_file_opened = (sdcard_file_close(&file) != 0);
					ble_update_status_and_dor(ST_DISK_FULL, total_days_of_records);
					put_device_in_power_save_mode();
					return;
				} else if (w_res != store_block_size) {
					// Something wrong while writing on SD Card
					LOG_ERR("Nbr bytes written: %d (store_block_size: %d)", w_res, store_block_size);
					is_file_opened = (sdcard_file_close(&file) != 0);
					ble_update_status_and_dor(ST_ERROR, total_days_of_records);
					put_device_in_power_save_mode();
					return;
				} 
				else {
					diff_t = difftime(end_t, start_time_ts.tv_sec);
					LOG_INF("Now: %d.%d.%d %02d:%02d:%02d, Execution time: %.0f [s]", tm_.tm_mday, tm_.tm_mon+1, tm_.tm_year+1900, tm_.tm_hour, tm_.tm_min, tm_.tm_sec, diff_t);
					if (diff_t >= CONFIG_STORAGE_TIME_DIFF_THRESHOLD)
					{
						LOG_WRN("Execution time exceed %d [s]", CONFIG_STORAGE_TIME_DIFF_THRESHOLD);
						sdcard_file_close(&file);
						is_file_opened = sdcard_file_setup_and_open(&file, ++file_idx);
						time(&start_time_ts.tv_sec);
					}
				}

				// Process the number of days of recording
				localtime_r(&end_t, &tm_);
				uint8_t day = tm_.tm_mday;
				uint8_t mon = tm_.tm_mon;
				int year = tm_.tm_year;
				if (day < start_day) {
					// borrow days from february
					if (mon == 2) {
						//  check whether year is a leap year
						if ((year % 4 == 0 && year % 100 != 0) || (year % 400 == 0)) {
							day += 29;
						} else {
							day += 28;
						}   
					}
					// borrow days from April or June or September or November
					else if (mon == 4 || mon == 6 || mon == 9 || mon == 11) 
					{
						day += 30; 
					}

					// borrow days from Jan or Mar or May or July or Aug or Oct or Dec
					else
					{
						day += 31;
					}
				}
				total_days_of_records = day - start_day;
				ble_update_status_and_dor(main_state, total_days_of_records);
			}
		} else {
			k_msleep(500);
		}
	}

	ble_update_status_and_dor(main_state, total_days_of_records);

	// Giving start's semaphore if thread could start
	k_sem_give(&thread_store_busy_sem);
	LOG_WRN("------------ Store to File Thread ended ... ------------\n");
}
