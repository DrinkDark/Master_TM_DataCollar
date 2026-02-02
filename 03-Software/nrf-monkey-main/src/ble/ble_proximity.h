#ifndef _BLE_PROXIMITY_H_
#define _BLE_PROXIMITY_H_

#include <zephyr/types.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <bluetooth/scan.h>

// Semaphores
extern struct k_sem thread_proximity_store_busy_sem;

// Global variables

/**
 * @brief Proximity file header (11 bytes total).
 * 
 * Written once at the start of a new .DAT file to provide metadata.
 */
struct proximity_file_header {
    uint16_t file_identifier;   // 0x0B1E for proximity file (used to verify file type)
    uint32_t  firmware_version;
    uint8_t  logger_id;
    uint32_t start_time;        // Reference Unix Epoch (UTC) when recording began
} __packed;

/**
 * @brief Individual device detection record (15 bytes total).
 * 
 * Represents a single "ping" or detection of a nearby BLE device.
 */
struct proximity_device_info {
    uint32_t timestamp; // Time of detection (Unix Epoch)
    uint8_t addr[6];    // Raw 48-bit Bluetooth MAC address
    int16_t device_number;
    int8_t rssi;
    uint8_t  days_of_recording;
    uint8_t  system_status;
} __packed; 

void init_scanning(void); 
int start_scanning(void);
int stop_scanning(void);

int find_device_number_in_adv_data(const char *name);

void proximity_flush_handler(struct k_work *work);

void ble_enable_proximity_detection(void);
void ble_disable_proximity_detection(void);
bool ble_proximity_is_busy(void);
void ble_proximity_thread_store_to_file(void);
#endif // _BLE_PROXIMITY_H_
