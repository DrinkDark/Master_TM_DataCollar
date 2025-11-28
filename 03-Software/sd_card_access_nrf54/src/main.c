/*
 * Copyright (c) 2019 Tavish Naruka <tavishnaruka@gmail.com>
 * Copyright (c) 2023 Nordic Semiconductor ASA
 * Copyright (c) 2023 Antmicro <www.antmicro.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Sample which uses the filesystem API and SDHC driver */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/fs/fs.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/device_runtime.h>
#include <zephyr/posix/time.h>
#include <zephyr/storage/disk_access.h>

#include <errno.h>
#if defined(CONFIG_FAT_FILESYSTEM_ELM)
    #include <ff.h>
#endif

#include "define.h"
#include "hal/gpio_hal.h"
#include "hal/btn_hal.h"
#if defined(CONFIG_FILE_SYSTEM)
    #include "fatfs/sdcard.h"
#endif 
#include "mic/t5848.h"

#if defined(CONFIG_FAT_FILESYSTEM_ELM)

LOG_MODULE_REGISTER(main, CONFIG_MAIN_LOG_LEVEL);

// Define stack size used by thread
#define THREAD_FATFS_STACKSIZE          4096
#define THREAD_MAIN_STACKSIZE           4096

// Set the priority of the thread
#define THREAD_FATFS_PRIORITY           7
#define THREAD_MAIN_PRIORITY            5

// Gpios & Hardware handling
#if !DT_NODE_HAS_STATUS(SPI_SD_NODE, okay)
    #warning "SPI SD Node is NOT okay"
#else
    static const struct device* spi_sd_dev = DEVICE_DT_GET(SPI_SD_NODE);
#endif

#if !DT_NODE_HAS_STATUS(SPI_MIC_NODE, okay)
    #warning "SPI MIC Node is NOT okay"
#else
	#define SPIOP      SPI_WORD_SET(8) | SPI_TRANSFER_MSB
	struct spi_dt_spec spi_mic_spec = SPI_DT_SPEC_GET(SPI_MIC_NODE, SPIOP, 0);
#endif

#else
    static const struct device* spi_dev = DEVICE_DT_GET(SPI_NODE);
#endif


/*
 *  Note the fatfs library is able to mount only strings inside _VOLUME_STRS
 *  in ffconf.h
 */
#if defined(CONFIG_DISK_DRIVER_MMC)
#define DISK_DRIVE_NAME "SD2"
#else
#define DISK_DRIVE_NAME "SD"
#endif

#define DISK_MOUNT_PT "/"DISK_DRIVE_NAME":"
#define FS_RET_OK FR_OK

FATFS main_fat_fs;
/* mounting info */
struct fs_mount_t main_mp = {
    .type = FS_FATFS,
    .fs_data = &main_fat_fs,
};

#endif

volatile uint8_t hot_reset;
volatile bool must_be_in_power_saving_mode;

bool is_sd_mic_gpio_set;
bool is_main_thread_initialized;

K_SEM_DEFINE(low_energy_mode_sem, 0, 1);

// SD Card & Mic Power GPIO handlers
#if DT_NODE_HAS_STATUS(SD_MIC_ENABLE_NODE, okay)
    struct gpio_dt_spec sd_mic_gpio = GPIO_DT_SPEC_GET(SD_MIC_ENABLE_NODE, gpios);

    static bool init_sd_mic_gpio(void)
    {
        static bool sd_mic_gpio_init = true;
        int ret;
        if (sd_mic_gpio_init)
        {
            if (!device_is_ready(sd_mic_gpio.port)) 
            {
                LOG_ERR("%s is not ready", sd_mic_gpio.port->name);
                return false;
            }

            ret = gpio_pin_configure_dt(&sd_mic_gpio, GPIO_OUTPUT_INACTIVE /* GPIO_OUTPUT_ACTIVE */);
            if (ret < 0) 
            {
                LOG_ERR("Failed to configure %s pin %d: %d", sd_mic_gpio.port->name, sd_mic_gpio.pin, ret);
                return false;
            }
            LOG_INF("GPIO %d configuration completed", sd_mic_gpio.pin);
            is_sd_mic_gpio_set = false; //true;
            sd_mic_gpio_init = false;
            disable_hardware_drivers();

            // #if !DT_NODE_HAS_STATUS(I2S_NODE, okay)
            // {
            //     disconnect_i2s_gpio();
            // }
            // #endif

            #if !DT_NODE_HAS_STATUS(SPI_SD_NODE, okay)
            {
                disconnect_spi_gpio();
            }
            #endif

        }
        return true;
    }
#else
    struct gpio_dt_spec sd_mic_gpio = NULL;
    static bool init_sd_mic_gpio(void) { return true; }
#endif // #if DT_NODE_HAS_STATUS(SD_MIC_ENABLE_NODE, okay)

void set_power_on_sd_and_mic(bool active)
{
	#if DT_NODE_HAS_STATUS(SD_MIC_ENABLE_NODE, okay)
	{
		int ret;
		if (active) {
			ret = gpio_pin_set_dt(&sd_mic_gpio, 1);
			if (ret == 0) {
				LOG_DBG("Power on mic and SD card is set !");
				is_sd_mic_gpio_set = true;
			} else {
				LOG_ERR("gpio_pin_set_dt(&sd_mic_gpio, 1) FAILED ! Error: %d", ret);
			}
		} else {
			ret = gpio_pin_set_dt(&sd_mic_gpio, 0);
			if (ret == 0) {
				LOG_DBG("Power on mic and SD card is OFF !");
				is_sd_mic_gpio_set = false;
			} else {
				LOG_ERR("gpio_pin_set_dt(&sd_mic_gpio, 1) FAILED ! Error: %d", ret);
			}
		}
	}
	#endif // #if DT_NODE_HAS_STATUS(SD_MIC_ENABLE_NODE, okay)	
}

static bool handle_spi_sd_action(bool active)
{
	#if DT_NODE_HAS_STATUS(SPI_SD_NODE, okay)
	{
		uint8_t spi_sd_state; 
		int ret = pm_device_state_get(spi_sd_dev, &spi_sd_state);
		if (ret == 0) {
			LOG_INF("SD SPI state: %s", pm_device_state_str(spi_sd_state));
			if (active) {
				if (spi_sd_state == PM_DEVICE_STATE_SUSPENDING || spi_sd_state == PM_DEVICE_STATE_SUSPENDED) {
					LOG_DBG("Resume the SD SPI ...");
					ret = pm_device_action_run(spi_sd_dev, PM_DEVICE_ACTION_RESUME);
					if (ret != 0) {
						LOG_ERR("pm_device_action_run(spi_sd_dev, PM_DEVICE_ACTION_RESUME) FAILED ! Error: %d", ret);
					}
				} else if (spi_sd_state == PM_DEVICE_STATE_ACTIVE) {
					return true;
				}
			} else {
				if (spi_sd_state == PM_DEVICE_STATE_ACTIVE) {
					LOG_DBG("Suspend the SD SPI ...");
					ret = pm_device_action_run(spi_sd_dev, PM_DEVICE_ACTION_SUSPEND);
					if (ret != 0) {
						LOG_ERR("pm_device_action_run(spi_sd_dev, PM_DEVICE_ACTION_SUSPEND) FAILED ! Error: %d", ret);
					}
				} else if (spi_sd_state == PM_DEVICE_STATE_SUSPENDED) {
					gpio_hal_disconnect_spi_gpio();
					return true;
				}
			}
		} else {
			if (ret == -ENOSYS) {
				LOG_ERR("Function not implemented for SD SPI !");
				return true;
			} else
				LOG_ERR("pm_device_state_get(spi_sd_dev, &spi_sd_state) FAILED ! Error: %d", ret);
		}
		return false;
	}
	#else
		return true;
	#endif // #if DT_NODE_HAS_STATUS(SPI_SD_NODE, okay)
}

static bool handle_spi_mic_action(bool active)
{
	return true;
}

void enable_hardware_drivers(void) 
{
	set_power_on_sd_and_mic(true);
	k_msleep(250);

	LOG_DBG("Enabling SD SPI ...");
	while (!handle_spi_sd_action(true)) {
		k_msleep(500);
	}
	LOG_DBG("SPI SD is enabled !");

	LOG_DBG("Enabling MIC SPI ...");
	while (!handle_spi_mic_action(true)) {
		k_msleep(500);
	}
	LOG_DBG("SPI MIC is enabled !");

	// /* PM is not supported by I2S driver */
	// LOG_DBG("Enabling I2S ...");
	// while (!handle_i2s_action(true)) {
	// 	k_msleep(500);
	// }
	// LOG_DBG("I2S is enabled !");

	// Must wait some time to be sure that all threads have detected the power on hardware
	k_msleep(CONFIG_DELAY_ON_HARDWARE_POWERED_ON);
}

void disable_hardware_drivers(void)
{
	LOG_DBG("Disabling SD SPI ...");
	while (!handle_spi_sd_action(false)) {
		k_msleep(500);
	}
	LOG_DBG("SD SPI is disabled !");

	LOG_DBG("Disabling MIC SPI ...");
	while (!handle_spi_sd_action(false)) {
		k_msleep(500);
	}
	LOG_DBG("MIC SPI is disabled !");

	// /* PM is not supported by I2S driver */
	// LOG_DBG("Disabling I2S ...");
	// while (!handle_i2s_action(false)) {
	// 	k_msleep(500);
	// }
	// LOG_DBG("I2S is disabled !");

	set_power_on_sd_and_mic(false);
}

void system_reset(void)
{
	LOG_WRN("NVIC_SystemReset() will be called soon !");

	// // Stop any BLE activity on Network core
	// LOG_DBG("Stopping any BLE activity ...");
	// ble_thread_running = false;

	// // Checking BLE activity ...
	// k_sem_take(&thread_ble_busy_sem, K_FOREVER);
	// LOG_INF("No more BLE activity running");

	// Waiting for end of fat fs thread
	k_sem_take(&thread_fatfs_busy_sem, K_FOREVER);

	// Update Hot Reset Flag
	hot_reset = CONFIG_HOT_RESET_VAL;
	// k_msleep(2000);

	// Applying the Software Reset
	NVIC_SystemReset();
}

static void main_thread(void) 
{
	// Taking semaphores of all other threads
	k_sem_take(&thread_fatfs_busy_sem, K_NO_WAIT);

	// Initialize global variables that MUST be initialized with a HOT reset
	is_sd_mic_gpio_set 			    = false;
	is_main_thread_initialized 	    = false;

	gpio_hal_disconnect_all_unused();

	if (hot_reset == CONFIG_HOT_RESET_VAL) {
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
		#endif
	} else {
		// Initialize NON Init variables
    	must_be_in_power_saving_mode    = false;

		// ASCII Art Generator: http://patorjk.com/software/taag
		// Font: Big
		LOG_INF("___________________________________________________________");
		LOG_INF("  __  __       _     _ _       _____  ");              
		LOG_INF(" |  \\/  |     | |   (_) |     / ____| ");           
		LOG_INF(" | \\  / | ___ | |__  _| | ___| (___   ___ _ __  ___ ");
		LOG_INF(" | |\\/| |/ _ \\| '_ \\| | |/ _ \\\\___ \\ / _ \\ '_ \\/ __|");
		LOG_INF(" | |  | | (_) | |_) | | |  __/____) |  __/ | | \\__ \\");
		LOG_INF(" |_|  |_|\\___/|_.__/|_|_|\\___|_____/ \\___|_| |_|___/");
		LOG_INF("___________________________________________________________");
		LOG_INF("");
		#ifdef CONFIG_COMPILE_FOR_MOBILESENS_PCB
			LOG_INF("Hardware: MobileSens PCB");
		#else
			LOG_INF("Hardware: PCA10156");
		#endif
		LOG_INF("___________________________________________________________");
		LOG_DBG("Starting MobileSens App ... (%s)", CONFIG_BOARD);
	}

	// #if DT_NODE_HAS_STATUS_OKAY(BTN0_NODE) || DT_NODE_HAS_STATUS_OKAY(BTN1_NODE) || DT_NODE_HAS_STATUS_OKAY(BTN2_NODE) || DT_NODE_HAS_STATUS_OKAY(BTN3_NODE)
	// {
	// 	if (!btn_hal_buttons_init()) {
	// 		LOG_WRN("Could NOT initialize some Button's handler...");
	// 	}
	// }
	// #endif // #if DT_NODE_HAS_STATUS_OKAY(BTN0_NODE) || DT_NODE_HAS_STATUS_OKAY(BTN1_NODE) || DT_NODE_HAS_STATUS_OKAY(BTN2_NODE) || DT_NODE_HAS_STATUS_OKAY(BTN3_NODE)

	// #if DT_NODE_HAS_STATUS(SD_MIC_ENABLE_NODE, okay)
	// {
	// 	if (!init_sd_mic_gpio()) {
	// 		LOG_ERR("init_sd_mic_gpio() FAILED !");
	// 		return;
	// 	}

	// 	// For now on, we just enable the hardware !
	// 	enable_hardware_drivers();
	// }
	// #endif // #if DT_NODE_HAS_STATUS(SD_MIC_ENABLE_NODE, okay)

	// trigger_command(i2s_dev_rx, i2s_dev_tx, I2S_TRIGGER_START);

	// // Giving semaphores of all other threads
	// k_sem_give(&thread_fatfs_busy_sem);

	// while (!must_be_in_power_saving_mode) {
	// 	LOG_DBG("low_energy_mode_sem.count: %d", low_energy_mode_sem.count);
	// 	LOG_DBG("low_energy_mode_sem.limit: %d", low_energy_mode_sem.limit);

	// 	int ret = k_sem_take(&low_energy_mode_sem, K_FOREVER);
	// 	switch (ret)
	// 	{
	// 		case -EBUSY:
	// 			LOG_WRN("k_sem_take(&low_energy_mode_sem, K_FOREVER) returns EBUSY");
	// 			break;

	// 		case -EAGAIN:
	// 			LOG_WRN("k_sem_take(&low_energy_mode_sem, K_FOREVER) returns EAGAIN");
	// 			break;

	// 		default:
	// 			LOG_DBG("k_sem_take(&low_energy_mode_sem, K_FOREVER) returns %d", ret);
	// 			break;
	// 	}
	// 	LOG_DBG("low_energy_mode_sem.count: %d", low_energy_mode_sem.count);
	// 	LOG_WRN("Handling Power Saving Mode ! ...");
	// 	must_be_in_power_saving_mode = true;

	// 	void *mem_block;
	// 	size_t size;

	// 	ret = i2s_read(i2s_dev_rx, &mem_block, &size);
	// 	if (ret < 0) {
	// 		LOG_ERR("I2S read failed: %d", ret);
	// 		break;
	// 	}

	// 	audio_msg_t msg;
	// 	msg.mem_block = mem_block;
	// 	msg.size = size;

	// 	ret = k_msgq_put(&audio_msgq, &msg, K_NO_WAIT);
		
	// 	if (ret != 0) {
	// 		LOG_WRN("Drop packet! SD thread too slow.");
	// 		k_mem_slab_free(&i2s_mem_slab, &mem_block);
	// 	}


	// 	// Workaround to disable completely FAT_fs
	// 	if (is_sd_mic_gpio_set) {
	// 		// Stop recording
	// 		// LOG_DBG("Stop recording ...");
	// 		// recorder_disable_record();

	// 		system_reset();
	// 		trigger_command(i2s_dev_rx, i2s_dev_tx, I2S_TRIGGER_DROP);

	// 		// k_msleep(3000);
	// 		// hot_reset = CONFIG_HOT_RESET_VAL;
	// 		// NVIC_SystemReset();
	// 	}
	// }

	// // Around GPIOs
	// // Will be done only if Low Batt detected -> moved in low_batt_debounced() method
	// // gpio_hal_disconnect_low_batt_gpio();

	// // No more needed, it's done in `disable_hardware_drivers()` method !
	// // gpio_hal_disconnect_i2s_gpio();
	// // gpio_hal_disconnect_spi_gpio();
	// LOG_WRN("------------ Main Thread ended ... ------------\n");

	LOG_INF("Tests started !");

	const struct t5848_aad_d_conf conf = {
		T5848_AAD_SELECT_D1,
		T5848_AAD_D_ALGO_SEL_REL,
		T5848_AAD_D_FLOOR_65dB,
		T5848_AAD_D_REL_PULSE_MIN_10ms,
		T5848_AAD_D_ABS_PULSE_MIN_48ms,
		T5848_AAD_D_ABS_THR_85dB,
		T5848_AAD_D_REL_THR_6dB
	};

	static struct t5848_address_data_pair reg_data_pairs[T5848_CONFIG_PAIRS_D];
	int count = t5848_generate_aad_d_pair(&conf, reg_data_pairs);

	t5848_generate_bit_pattern(reg_data_pairs, count, &spi_mic_spec);

	LOG_DBG("Disabling MIC SPI ...");
	while (!handle_spi_mic_action(false)) {
		k_msleep(500);
	}
	LOG_DBG("MIC SPI is disabled !");

	LOG_INF("Tests ended !");
}

// Define and initialize the threads
K_THREAD_DEFINE(thread_main_id,  THREAD_MAIN_STACKSIZE,  main_thread, NULL, NULL, NULL, THREAD_MAIN_PRIORITY, 0, 0);
K_THREAD_DEFINE(thread_fatfs_id, THREAD_FATFS_STACKSIZE, sdcard_thread_fatfs_mount, &main_mp, NULL, NULL, THREAD_FATFS_PRIORITY, 0, 0);
