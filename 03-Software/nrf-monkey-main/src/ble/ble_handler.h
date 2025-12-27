#ifndef _BLE_HANDLER_H_
#define _BLE_HANDLER_H_

#include <zephyr/types.h>

#include "snes.h"

// Semaphores
extern struct k_sem thread_ble_busy_sem;
extern struct k_sem thread_proximity_store_busy_sem;

// Global variables
extern struct bt_conn* current_conn;
extern struct bt_conn* auth_conn;

extern struct gpio_dt_spec collar_burn_gpio;

struct ble_data_t {
    void* fifo_reserved;
    uint8_t data[CONFIG_BT_SNES_BUFFER_SIZE];
    uint16_t len;
};

struct proximity_device_info {
    uint32_t timestamp;
    uint8_t addr[BT_ADDR_LE_STR_LEN];
    int32_t device_number;
    int8_t rssi;
    int8_t tx_power;
} __packed;

extern bool ble_thread_running;

void ble_thread_init(void);

void ble_update_status_and_dor(uint8_t status, uint8_t nbr);
void ble_update_device_id_char_val(void);
void ble_update_mic_gain_char_val(void);
void ble_disconnect(void);

#endif // _BLE_HANDLER_H_
