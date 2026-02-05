#ifndef _SDCARD_H_
#define _SDCARD_H_

#include <zephyr/drivers/gpio.h>
#include <zephyr/fs/fs.h>
#include <zephyr/kernel.h>

#include <ff.h>

// Global variables
extern struct k_sem card_present_sem;
extern struct k_sem file_access_sem;
extern struct k_sem thread_fatfs_busy_sem;

extern struct gpio_dt_spec card_detect_gpio;

extern const char* mount_pt;
extern const char* disk_pdrv;
extern const char* file_name;
extern const char* file_ext;


// Card Detect Gpio related
void cd0_handler(const struct device* dev, struct gpio_callback* cb, uint32_t pins);
bool init_sd_detect_gpio(void);

// Intialization of fatfs
int sdcard_init(const char* pdrv);
void sdcard_init_fs(struct fs_mount_t* mp, FATFS* fs_data);
int sdcard_disc_init(void);

// Try to mount the given path to mount point mp
int sdcard_mount(struct fs_mount_t* mp, const char* path);
int sdcard_unmount(struct fs_mount_t* mp);
void sdcard_off_and_reset(struct fs_mount_t* mp, bool waiting_for_sdcard);

#ifdef CONFIG_STORAGE_LIST_FILES_ON_CARD
    // Method for listing from a given path
    int sdcard_lsdir(const char* path);
#endif // #ifdef CONFIG_STORAGE_LIST_FILES_ON_CARD

// FAT fs around files
void sdcard_file_init(struct fs_file_t* zfp);
bool sdcard_file_setup_and_open(struct fs_file_t* zfp, const char* file_name, int index);
int sdcard_file_close(struct fs_file_t* zfp);
int sdcard_file_sync(struct fs_file_t* zfp);

ssize_t sdcard_write(struct fs_file_t* zfp, const void* ptr, size_t size);

bool sdcard_file_exists(const char* file_name);
bool sdcard_is_mounted(struct fs_mount_t* mp, const char* name);
bool sdcard_is_ready();

// Around Thread
void sdcard_thread_fatfs_mount(struct fs_mount_t* mp_thread);


#endif // _SDCARD_H_
