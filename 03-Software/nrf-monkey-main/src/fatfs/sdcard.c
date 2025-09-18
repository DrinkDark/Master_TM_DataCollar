/*
 * Copyright (c) 2023, HES-SO Valais-Wallis, HEI, Sion
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "sdcard.h"

#include <zephyr/logging/log.h>
#include <zephyr/storage/disk_access.h>
#include <zephyr/sd/sd.h>

#include <errno.h>
#include <stdio.h>
#include <time.h>

#include <soc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sdhc.h>
#include <hal/nrf_gpio.h>
#include <zephyr/pm/pm.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/policy.h>

#include "../ble/ble_handler.h"
#include "../define.h"

#define SD_CD0_NODE        	DT_ALIAS(cd0)
#define SDHC_NODE			DT_NODELABEL(sdhc0)


LOG_MODULE_REGISTER(sdcard, CONFIG_SD_CARD_LOG_LEVEL);

// Defines the Semaphores
K_SEM_DEFINE(card_present_sem, 1, 1);
K_SEM_DEFINE(file_access_sem, 1, 1);
K_SEM_DEFINE(thread_fatfs_busy_sem, 1, 1);

/* Note the fatfs library is able to mount only strings inside _VOLUME_STRS in ffconf.h */
const char* mount_pt = CONFIG_STORAGE_MOUNT_POINT;
const char* disk_pdrv = CONFIG_STORAGE_DISK_POINTER_NAME;
const char* file_name = CONFIG_STORAGE_FILE_NAME;
const char* file_ext  = CONFIG_STORAGE_FILE_EXTENSION;

static bool is_sdcard_ready;

#if !DT_NODE_HAS_STATUS(SDHC_NODE, okay)
	warning "Can not access to sdhc dts node"
#else
	static const struct device* sdhc_dev = DEVICE_DT_GET(SDHC_NODE);
#endif // #if !DT_NODE_HAS_STATUS(SDHC_NODE, okay)

#if DT_NODE_HAS_STATUS(SD_CD0_NODE, okay)
struct gpio_dt_spec card_detect_gpio = GPIO_DT_SPEC_GET(SD_CD0_NODE, gpios);
static struct k_work_delayable sd_detect_work;

// static struct spi_dt_spec spi = SPI_DT_SPEC_INST_GET(4, SPI_OP_MODE_MASTER | SPI_MODE_CPOL | SPI_MODE_CPHA | SPI_WORD_SET(8) | SPI_TRANSFER_MSB, 0U);
extern int disk_access_unregister(struct disk_info *disk);
extern struct disk_info *disk_access_get_di(const char *name);

static void sd_detect_debounced(struct k_work* work)
{
	ARG_UNUSED(work);
	if (gpio_pin_get_dt(&card_detect_gpio) == 0)
	{
		LOG_WRN("SD Card removed!");
		k_sem_take(&card_present_sem, K_NO_WAIT);
	}
	else
	{
		LOG_INF("SD Card inserted");
		k_sem_give(&card_present_sem);
	}
}
static K_WORK_DELAYABLE_DEFINE(sd_detect_work, sd_detect_debounced);

DWORD get_fattime(void)
{
	static struct tm tm_;
	static time_t now;

	time(&now);
	localtime_r(&now, &tm_);
	return 	(((WORD)(tm_.tm_year - 80) << 25 | ((DWORD) (tm_.tm_mon+1) << 21) | ((DWORD)tm_.tm_mday << 16)) |
			 ((WORD)(tm_.tm_hour * 2048U | tm_.tm_min * 32U)));
}

void cd0_handler(const struct device* dev, struct gpio_callback* cb, uint32_t pins)
{
	if ((CONFIG_STORAGE_CD_DEBOUNCE_MS) != 0) {
		k_work_reschedule(&sd_detect_work, K_MSEC(CONFIG_STORAGE_CD_DEBOUNCE_MS));
	} else {
		sd_detect_debounced(NULL);
	}
}
#else
void cd0_handler(const struct device* dev, struct gpio_callback* cb, uint32_t pins) {}
#endif

static bool is_card_present() {
	#ifdef CONFIG_STORAGE_USE_SDHC_NODE_FEATURE
	{
		struct sdhc_host_props sdhc_props;
		int err = sdhc_get_host_props(sdhc_dev, &sdhc_props);
		LOG_INF("sdhc_get_host_props() -> %d", err);
		if (err == 0) {
			LOG_DBG("sdhc is SPI ? %s", sdhc_props.is_spi ? "YES":"NO");
		}
		
		int present = sdhc_card_present(sdhc_dev);
		if (present == 0) {
			LOG_WRN("Card not present ! (present: %d)", present);
		} else if (present == 1) {
			LOG_INF("Card present ! (present: %d)", present);
		} else {
			LOG_ERR("Failed detecting card presentness ! (present: %d)", present);
		}
		return (present == 1);
	}
	#else
	{
		LOG_DBG("Card Detect GPIO: %d", gpio_pin_get_dt(&card_detect_gpio));
		return (gpio_pin_get_dt(&card_detect_gpio) != 0);
	}
	#endif // #ifdef CONFIG_STORAGE_USE_SDHC_NODE_FEATURE
}

static void init_fs_data(FATFS* fs_data) 
{
	fs_data->fs_type		= 0;			// Filesystem type (0:not mounted)
	fs_data->pdrv			= 0;			// Associated physical drive
	fs_data->n_fats			= 0;			// Number of FATs (1 or 2)
	fs_data->wflag			= 0;			// win[] flag (b0:dirty)
	fs_data->fsi_flag		= 0;			// FSINFO flags (b7:disabled, b0:dirty)
	fs_data->id				= 0;			// File system mount ID
	fs_data->n_rootdir		= 0;			// Number of root directory entries (FAT12/16)
	fs_data->csize			= 0;			// Cluster size [sectors]
	#if FF_MAX_SS != FF_MIN_SS
		fs_data->ssize		= 0;			// Sector size (512, 1024, 2048 or 4096)
	#endif
	#if FF_USE_LFN
		fs_data->lfnbuf		= NULL;			// LFN working buffer
	#endif
	#if FF_FS_EXFAT
		fs_data->dirbuf		= NULL;			// Directory entry block scratchpad buffer for exFAT
	#endif
	#if FF_FS_REENTRANT
		fs_data->sobj		= NULL;			// Identifier of sync object
	#endif
	#if !FF_FS_READONLY
		fs_data->last_clst	= 0;			// Last allocated cluster
		fs_data->free_clst	= 0;			// Number of free clusters
	#endif
	#if FF_FS_RPATH
		fs_data->cdir		= 0;			// Current directory start cluster (0:root)
		#if FF_FS_EXFAT
			fs_data->cdc_scl	= 0;		// Containing directory start cluster (invalid when cdir is 0)
			fs_data->cdc_size	= 0;		// b31-b8:Size of containing directory, b7-b0: Chain status
			fs_data->cdc_ofs	= 0;		// Offset in the containing directory (invalid when cdir is 0)
		#endif
	#endif
		fs_data->n_fatent	= 0;			// Number of FAT entries (number of clusters + 2)
		fs_data->fsize		= 0;			// Size of an FAT [sectors]
		fs_data->volbase	= 0;			// Volume base sector
		fs_data->fatbase	= 0;			// FAT base sector
		fs_data->dirbase	= 0;			// Root directory base sector/cluster
		fs_data->database	= 0;			// Data base sector
	#if FF_FS_EXFAT
		fs_data->bitbase	= 0;			// Allocation bitmap base sector
	#endif
		fs_data->winsect	= 0;			// Current sector appearing in the win[]
		memset(fs_data->win, 0, FF_MAX_SS);	// Disk access window for Directory, FAT (and file data at tiny cfg)
}

bool init_sd_detect_gpio(void)
{
	#if DT_NODE_HAS_STATUS(SD_CD0_NODE, okay)
	{
		int ret;
		static bool sd_detect_gpio_init = true;
		static struct gpio_callback cd0_callback;

		if (sd_detect_gpio_init)
		{
			if (!device_is_ready(card_detect_gpio.port)) 
			{
				LOG_ERR("%s is not ready", card_detect_gpio.port->name);
				return false;
			}

			ret = gpio_pin_configure_dt(&card_detect_gpio, GPIO_INPUT);
			if (ret < 0) 
			{
				LOG_ERR("Failed to configure %s pin %d: %d", card_detect_gpio.port->name, card_detect_gpio.pin, ret);
				return false;
			}

			ret = gpio_pin_interrupt_configure_dt(&card_detect_gpio, GPIO_INT_TRIG_BOTH);
			if (ret < 0) 
			{
				LOG_ERR("Failed to configure interrupt on %s pin %d: %d", card_detect_gpio.port->name, card_detect_gpio.pin, ret);
				return false;
			}

			gpio_init_callback(&cd0_callback, cd0_handler, BIT(card_detect_gpio.pin));
			gpio_add_callback(card_detect_gpio.port, &cd0_callback);
			sd_detect_gpio_init = false;
			LOG_INF("GPIO %d configuration completed", card_detect_gpio.pin);
		}
	}
	#endif // #if DT_NODE_HAS_STATUS(SD_CD0_NODE, okay)
	return true;
}


int sdcard_init(const char* pdrv) {
	int res = FR_OK;
	if (init_sd_detect_gpio()) 
		LOG_INF("init_sd_detect_gpio() Completed!");
	else
	{
		res = -EIO;
		LOG_WRN("init_sd_detect_gpio() FAILED! error: %d", res);
	}
	
	LOG_DBG("sdcard_init() returns %d", res);
    return res;
}

void sdcard_init_fs(struct fs_mount_t* mp, FATFS* fs_data)
{
	// /**
	//  * @brief File system mount info structure
	//  *
	//  * @param node Entry for the fs_mount_list list
	//  * @param type File system type
	//  * @param mnt_point Mount point directory name (ex: "/fatfs")
	//  * @param fs_data Pointer to file system specific data
	//  * @param storage_dev Pointer to backend storage device
	//  * @param mountp_len Length of Mount point string
	//  * @param fs Pointer to File system interface of the mount point
	//  * @param flags Mount flags
	//  */
	/*
	struct fs_mount_t {
		sys_dnode_t node;
		int type;
		const char *mnt_point;
		void *fs_data;
		void *storage_dev;
		// fields filled by file system core
		size_t mountp_len;
		const struct fs_file_system_t *fs;
		uint8_t flags;
	};
	*/
	mp->node.head 	= NULL;
	mp->node.next 	= NULL;
	mp->node.prev 	= NULL;
	mp->node.tail 	= NULL;

	mp->mnt_point	= NULL;
	
	init_fs_data(mp->fs_data);
	mp->fs_data		= NULL;
	
	mp->storage_dev	= NULL;
	mp->mountp_len	= 0;
	mp->fs			= NULL;
	mp->flags		= 0;

	init_fs_data(fs_data);
	mp->type		= FS_FATFS;
	mp->fs_data		= fs_data;
}

int sdcard_disc_init(void) 
{
	/* raw disk i/o */
	static const char* disk_pdrv = "SD";
	uint64_t memory_size_mb;
	uint32_t block_count;
	uint32_t block_size;

	// Re initialize the SD Card
	int status = disk_access_status(disk_pdrv);
	LOG_INF("Disk Status: %d", status);

	int res = disk_access_init(disk_pdrv);
	if (res == FR_OK) {
		res = disk_access_ioctl(disk_pdrv, DISK_IOCTL_GET_SECTOR_COUNT, &block_count);
		if (res == FR_OK) {
			LOG_DBG("Block count %u", block_count);
			res = disk_access_ioctl(disk_pdrv, DISK_IOCTL_GET_SECTOR_SIZE, &block_size);
			if (res == FR_OK) {
				LOG_DBG("Sector size %u", block_size);
				memory_size_mb = (uint64_t)block_count*  block_size;
				LOG_DBG("Memory Size(MB) %u\n", (uint32_t)(memory_size_mb >> 20));
			} else {
				LOG_ERR("Unable to get sector size. error: %d", res);
			}
		} else {
			LOG_ERR("Unable to get sector count. error: %d", res);
		}
	} else {
		LOG_ERR("Storage init ERROR! error: %d", res);
	}
	return res;
}


int sdcard_mount(struct fs_mount_t* mp, const char* path)
{
    mp->mnt_point = path;
    LOG_DBG("Mounting %s ...", path);
	return fs_mount(mp);
}

int sdcard_unmount(struct fs_mount_t* mp) 
{
	int res = fs_unmount(mp);
	LOG_DBG("sdcard_unmount() > %d", res);

	#if DT_NODE_HAS_STATUS(SD_CD0_NODE, okay)
	{
		res = sdhc_hw_reset(sdhc_dev);
		if (res) {
			LOG_ERR("sdhc_hw_reset(&sdhc_dev) FAILED ! (error: %d)", res);
		}
	}
	#endif // #if DT_NODE_HAS_STATUS(SD_CD0_NODE, okay)
	return res;
}

void sdcard_off_and_reset(struct fs_mount_t* mp, bool waiting_for_sdcard)
{
	if (sdcard_is_mounted(mp, mount_pt)) {
		sdcard_unmount(mp);
	}
	is_sdcard_ready     = false;

	sdcard_init_fs(mp, mp->fs_data);
	int err = clock_gettime(CLOCK_REALTIME, &hot_reset_ts);
	if (err != 0) {
		LOG_ERR("clock_gettime() FAILED!");
	}

	if (waiting_for_sdcard) {
		ble_update_status_and_dor(ST_WAIT_SD_CARD, total_days_of_records);
	} else {
		ble_update_status_and_dor(ST_INIT, total_days_of_records);
	}
	
	// Giving start's semaphore if thread could start
	k_sem_give(&thread_fatfs_busy_sem);
	LOG_WRN("------------ THREAD for FAT FS ended ! ------------");

	system_reset();
}

#ifdef CONFIG_STORAGE_LIST_FILES_ON_CARD
	int sdcard_lsdir(const char* path) {
		int res;
		struct fs_dir_t dirp;
		static struct fs_dirent entry;

		LOG_DBG("Listing dir %s ...", path);
		fs_dir_t_init(&dirp);

		/* Verify fs_opendir() */
		res = fs_opendir(&dirp, path);
		if (res) {
			LOG_ERR("Error opening dir %s [%d]\n", path, res);
			return res;
		}

		for (;;) {
			/* Verify fs_readdir() */
			res = fs_readdir(&dirp, &entry);

			/* entry.name[0] == 0 means end-of-dir */
			if (res || entry.name[0] == 0) {
				break;
			}

			if (entry.type == FS_DIR_ENTRY_DIR) {
				LOG_DBG("[DIR ] %s", entry.name);
			} else {
				LOG_DBG("[FILE] %s (size = %zu)", entry.name, entry.size);
			}
		}

		/* Verify fs_closedir() */
		fs_closedir(&dirp);
		LOG_DBG("Listing finished\n");

		return res;
	}
#endif // #ifdef CONFIG_STORAGE_LIST_FILES_ON_CARD

void sdcard_file_init(struct fs_file_t* zfp)
{
	fs_file_t_init(zfp);
}

static int sdcard_file_open(struct fs_file_t* zfp, const char* file_name, fs_mode_t flags) 
{
	LOG_DBG("Opening file \"%s\" ...", file_name);
	int res = fs_open(zfp, file_name, flags);
	if (res != FR_OK && res != -EBUSY) {
		LOG_ERR("fs_open(...) FAILED ! error: %d", res);
	} else {
		LOG_DBG("fs_open(...) completed !");
	}
    return FR_OK;
}

bool sdcard_file_setup_and_open(struct fs_file_t* zfp, int index)
{
	char f_name[20];
	memset(f_name, 0x00, 20);

	// Solved by reading https://devzone.nordicsemi.com/f/nordic-q-a/72654/how-to-set-the-date-and-time-to-the-file-on-sd-card-or-flash-disk-on-nrf52840
	// I had to change in zephyr_fatfs_config.h the following defines :
	// 81 > #undef FF_FS_NORTC
	// 82 > #define FF_FS_NORTC 0 (FF_FS_NORTC was 0 and I set it to 1)
	//
	// I had also to change in ffconf.h:
	// 45 > #define FF_USE_CHMOD	1	(it was set to 0)

	sprintf(f_name, "%s/%s_%03d.%s", mount_pt, file_name, index, file_ext);
	LOG_INF("file_name: %s", f_name);
	return (sdcard_file_open(zfp, f_name, FS_O_CREATE | FS_O_RDWR | FS_O_APPEND) == 0);
}

int sdcard_file_close(struct fs_file_t* zfp)
{
	int res = fs_close(zfp);
	LOG_DBG("sdcard_file_close() > %d", res);
	return res; 
}


ssize_t sdcard_write(struct fs_file_t* zfp, const void* ptr, size_t size) {
	int res = 0;

	// Checks if there is still available space on disk
	struct fs_statvfs sbuf;
	res = fs_statvfs(zfp->mp->mnt_point, &sbuf);
	if (res == 0) {
		// Param f_bsize:	Optimal transfer block size
		// Param f_frsize:	Allocation unit size
		// Param f_blocks:	Size of FS in f_frsize units
		// Param f_bfree:	Number of free blocks
		uint32_t tmp_blocks = (size / sbuf.f_frsize) + 1;
		// LOG_DBG("free blocks: %lu, bloc size: %lu, unit size: %lu, size of FS: %lu (size to write: %d, blocs: %u)", sbuf.f_bfree, sbuf.f_bsize, sbuf.f_frsize, sbuf.f_blocks, size, tmp_blocks);
		if (tmp_blocks > sbuf.f_bfree) {
			LOG_ERR("Not enough space on disk ! Blocks available: %lu, blocks to write: %u", sbuf.f_bfree, tmp_blocks);
			res = -ENOMEM;
		}
	}

	if (res == 0)
	{
		res = fs_write(zfp, ptr, size);
		if (res < 0) 
		{
			LOG_ERR("fs_write(...) FAILED ! error: %d", res);
			return res;
		}
	}
    return res;
}


bool sdcard_file_exists(const char* file_name) {
	struct fs_dirent dirent;
	int ret;

	ret = fs_stat(file_name, &dirent);
	if (ret < 0) {
		LOG_ERR("FAIL: stat %s: %d", file_name, ret);
		return false;
	}

	return true;
}

bool sdcard_is_mounted(struct fs_mount_t* mp, const char* path)
{
	int ret;
	struct fs_statvfs sbuf;

	ret = fs_statvfs(path, &sbuf);
	
	if (ret < 0) {
		if (ret == -ENOENT) {
			LOG_WRN("%s NOT mounted !", path);
		} else {
			LOG_ERR("FAIL: statvfs: %d", ret);
		}
		return false;
	}
	return true;
}

bool sdcard_is_ready() {
	return is_sdcard_ready;
}

void sdcard_thread_fatfs_mount(struct fs_mount_t* mp_thread) 
{
	int res;
	bool sd_card_initialized = false;

	LOG_DBG("Starting THREAD for FAT FS...");

	// RAM intialization after Hot Reset
	is_sdcard_ready = false;

	// Checking if thread could start
	k_sem_take(&thread_fatfs_busy_sem, K_FOREVER);
	
	// Turn on power on SD Card & mic (I2S)
	LOG_DBG("Waiting for power on SD Card ...");
	while (!is_sd_mic_gpio_set) {
		if (is_low_batt_detected || must_be_in_power_saving_mode) {
			ble_update_status_and_dor(main_state, total_days_of_records);
			LOG_WRN("------------ THREAD for FAT FS ended ! ------------");
			return;
		}
		k_msleep(2000);
	}

	LOG_INF("Starting FAT_FS ...");
	res = sdcard_init(disk_pdrv); 
	if (res != FR_OK) {
		LOG_ERR("sdcard_init() > %d", res);
		return;
	}

	// Trying to mount/unmount SD card
	do {
		while (!sd_card_initialized && !must_be_in_power_saving_mode) {
			if (is_card_present()) {
				res = sdcard_disc_init(); 
				if (res != FR_OK) {
					LOG_ERR("sdcard_disc_init() > %d", res);
				} else {
					LOG_INF("sdcard_disc_init() Completed\n");
					sd_card_initialized = true;
				}
			} else {
				LOG_WRN("No SD Card available...");
				ble_update_status_and_dor(ST_WAIT_SD_CARD, total_days_of_records);
			}
			k_msleep(2500);
		};

		if (is_card_present()) {
			if (!must_be_in_power_saving_mode && !sdcard_is_mounted(mp_thread, mount_pt)) {
				LOG_DBG("Mounting SD Card to %s ...", mount_pt);
				res = sdcard_mount(mp_thread, mount_pt);
				if (res != FR_OK) {
					LOG_ERR("sdcard_mount() > %d", res);
					sd_card_initialized = false;
				} else {
					LOG_INF("%s mounted !", mount_pt);
					#ifdef CONFIG_STORAGE_LIST_FILES_ON_CARD
						sdcard_lsdir(mount_pt);
					#endif // #ifdef CONFIG_STORAGE_LIST_FILES_ON_CARD
					is_sdcard_ready = true;
				}
			}
		} else {
			LOG_WRN("SD Card no more present...");
			sd_card_initialized = false;
			sdcard_off_and_reset(mp_thread, true);
			return;
		}

		k_msleep(2500);

		// LOG_DBG("is_sd_mic_gpio_set: %s, must_be_in_power_saving_mode: %s", is_sd_mic_gpio_set ? "YES":"NO", must_be_in_power_saving_mode ? "YES":"NO");
	} while(!must_be_in_power_saving_mode);

	// Giving start's semaphore if thread could start
	k_sem_give(&thread_fatfs_busy_sem);
	LOG_WRN("------------ THREAD for FAT FS ended ! ------------");
}
