/*
 * Copyright (c) 2023 HESSO-VS, HEI Sion
 */

#ifndef BT_SPEAK_NO_EVIL_SRV_H_
#define BT_SPEAK_NO_EVIL_SRV_H_

/**
 * @file
 * @defgroup bt_hei_snes HEI Speak No Evil (SNES) GATT Service
 * @{
 * @brief HEI Speak No Evil (SNES) GATT Service API.
 */

#include <zephyr/types.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief UUID of the SNES Service. 
 *  Helper macro to initialize a 128-bit UUID array value from the readable form
 *  of UUIDs, or encode 128-bit UUID values into advertising data
 *  Can be combined with BT_UUID_DECLARE_128 to declare a 128-bit UUID.
 *
 *  Example of how to declare the UUID `6E400001-B5A3-F393-E0A9-E50E24DCCA9E`
 * 
 *  This UUID is build on the acronym "HES-SO". The HES-SO Valais is registered in the Bluetooth SIF
 *  with the acronyme "University of Applied Science Valais/Haute Ecole Valaisanne".
 *  The middle part is its assigned number (0x025A) and the last part is build with the other acronym 
 *  used in that SIG registration: HEVs.
 * 
 *  The Base BT UUID: `0000*-4865-7673-025A-4845532D534F`, where `*` should be replaced by the 16 bits
 *  Service's UUID and characteristic's UUID
 *
 *  0x4F 0x53 0x2D 0x53 0x45 0x48 0x5A 0x02 0x73 0x76 0x65 0x48 0x00 0x00 0x00 0x00
 *    O    S    -    S    E    H    5A   02   s    v    e    H         ...
 **/
#define BT_UUID_SNES_VAL \
	BT_UUID_128_ENCODE(0x00000201, 0x4865, 0x7673, 0x025A, 0x4845532D534F)

/** @brief UUID of the CMD Characteristic. **/
#define BT_UUID_SNES_CMD_VAL \
	BT_UUID_128_ENCODE(0x00000202, 0x4865, 0x7673, 0x025A, 0x4845532D534F)

/** @brief UUID of the Status Characteristic. **/
#define BT_UUID_SNES_STATUS_VAL \
	BT_UUID_128_ENCODE(0x00000203, 0x4865, 0x7673, 0x025A, 0x4845532D534F)

/** @brief UUID of the Status Characteristic. **/
#define BT_UUID_SNES_DOR_VAL \
	BT_UUID_128_ENCODE(0x00000204, 0x4865, 0x7673, 0x025A, 0x4845532D534F)

/** @brief UUID of the Device Identifier. **/
#define BT_UUID_SNES_DEVICE_ID_VAL \
	BT_UUID_128_ENCODE(0x00000205, 0x4865, 0x7673, 0x025A, 0x4845532D534F)

/** @brief UUID of the Mic Input Gain. **/
#define BT_UUID_SNES_MIC_INPUT_GAIN_VAL \
	BT_UUID_128_ENCODE(0x00000206, 0x4865, 0x7673, 0x025A, 0x4845532D534F)

/** 
 * @brief UUID of the Mic AAD A params. 
 * 
 * Contains parameters for the AAD A level.
 * Data structure (2 bytes) :
 * 	- AAD A LPF value	: byte 0 (valid range -> 0x1 (4.4kHz) to 0x7 (1.1kHz))
 * 	- AAD A TH value 	: byte 1 (valide range -> 0x0 (60dB SPL) to 0xF (97.5dB SPL))
 * 
 * **/
#define BT_UUID_SNES_MIC_AAD_A_PARAM_VAL \
	BT_UUID_128_ENCODE(0x00000207, 0x4865, 0x7673, 0x025A, 0x4845532D534F)

/** * @brief UUID of the Mic AAD D params. 
 * Contains parameters for AAD Digital level (D1/D2).
 * Data structure (10 bytes) :
 * 	- ALGO_SEL      : byte 0 (0:None, 1:Rel, 2:Abs, 3:Both)
 * 	- FLOOR         : byte 1-2 (range 0x00F-0x7BC)
 * 	- REL_PULSE_MIN : byte 3-4 (range 0x000-0x12C)
 * 	- ABS_PULSE_MIN : byte 5-6 (range 0x000-0xDAC)
 * 	- ABS_THR       : byte 7-8 (range 0x00F-0x7BC)
 * 	- REL_THR       : byte 9   (range 0x24-0xFF)
 **/
#define BT_UUID_SNES_MIC_AAD_D1_PARAM_VAL \
    BT_UUID_128_ENCODE(0x00000208, 0x4865, 0x7673, 0x025A, 0x4845532D534F)


#define BT_UUID_SNES_SERVICE			BT_UUID_DECLARE_128(BT_UUID_SNES_VAL)
#define BT_UUID_SNES_CMD      			BT_UUID_DECLARE_128(BT_UUID_SNES_CMD_VAL)
#define BT_CUD_SNES_CMD					"Command"
#define BT_UUID_SNES_STATUS  			BT_UUID_DECLARE_128(BT_UUID_SNES_STATUS_VAL)
#define BT_CUD_SNES_STATUS				"Status"
#define BT_UUID_SNES_DAYS_OF_RECORDS	BT_UUID_DECLARE_128(BT_UUID_SNES_DOR_VAL)
#define BT_CUD_SNES_DAYS_OF_RECORDS		"Nbr of Days recording"
#define BT_UUID_SNES_DEVICE_IDENTIFIER	BT_UUID_DECLARE_128(BT_UUID_SNES_DEVICE_ID_VAL)
#define BT_CUD_SNES_DEVICE_IDENTIFIER	"Device Identifier"
#define BT_UUID_SNES_MIC_INPUT_GAIN		BT_UUID_DECLARE_128(BT_UUID_SNES_MIC_INPUT_GAIN_VAL)
#define BT_CUD_SNES_MIC_INPUT_GAIN		"Mic Input Gain"
#define BT_UUID_SNES_MIC_AAD_A_PARAM	BT_UUID_DECLARE_128(BT_UUID_SNES_MIC_AAD_A_PARAM_VAL)
#define BT_CUD_SNES_MIC_AAD_A_PARAM		"Mic AAD A params"
#define BT_UUID_SNES_MIC_AAD_D1_PARAM	BT_UUID_DECLARE_128(BT_UUID_SNES_MIC_AAD_D1_PARAM_VAL)
#define BT_CUD_SNES_MIC_AAD_D1_PARAM	"Mic AAD D1 params"

// /** @brief SNES send status. */
// enum bt_snes_send_status {
// 	/** Send notification enabled. */
// 	BT_SNES_SEND_STATUS_ENABLED,
// 	/** Send notification disabled. */
// 	BT_SNES_SEND_STATUS_DISABLED,
// };

/** @brief SNES notification status. */
enum bt_snes_notifification_status {
	/** Notification enabled. */
	BT_SNES_NOTIFICATION_ENABLED,
	/** Notification disabled. */
	BT_SNES_NOTIFICATION_DISABLED,
};


/** @brief Pointers to the callback functions for service events. */
struct bt_snes_cb {
	/** 
	 * @brief Command received callback.
	 *
	 * The data has been received as a write request on the SNES CMD Characteristics.
	 *
	 * @param[in] conn  Pointer to connection object that has received data.
	 * @param[in] data  Received data.
	 * @param[in] len   Length of received data.
	 */
	void (*cmd_received)(struct bt_conn *conn, const uint8_t *const data, uint16_t len);

	/** 
	 * @brief Data sent callback.
	 *
	 * The data has been sent as a notification and written on the SNES CMD Characteristic.
	 *
	 * @param[in] conn Pointer to connection object, or NULL if sent to all connected peers.
	 */
	void (*cmd_sent)(struct bt_conn *conn);

	/** 
	 * @brief Status state callback.
	 *
	 * Indicate the CCCD descriptor status of the SNES Status characteristic.
	 *
	 * @param[in] status Send notification status.
	 */
	void (*status_notif_changed)(enum bt_snes_notifification_status status);

	/** 
	 * @brief Days of Records state callback.
	 *
	 * Indicate the CCCD descriptor status of the SNES DOR characteristic.
	 *
	 * @param[in] status Send notification status.
	 */
	void (*dor_notif_changed)(enum bt_snes_notifification_status status);

	/**
	 * @brief Device Identifier Notification callback 
	 *
	 * Indicate the CCCD descriptor status of the SNES Device Identifier characteristic.
	 * 
	 * @param[in] did_notification Device Identifier Notification status (enable/disable).
	 */
	void (*did_notif_changed)(enum bt_snes_notifification_status did_notification);

	/**
	 * @brief Microphone Input Gain Notification callback 
	 *
	 * Indicate the CCCD descriptor status of the SNES Mic Input Gain characteristic.
	 * 
	 * @param[in] mig_notification Mic Input Gain Notification status (enable/disable).
	 */
	void (*mig_notif_changed)(enum bt_snes_notifification_status mig_notification);

		/**
	 * @brief Microphone AAD A params Notification callback 
	 *
	 * Indicate the CCCD descriptor status of the SNES Mic AAD A params characteristic.
	 * 
	 * @param[in] mig_notification Mic Input Gain Notification status (enable/disable).
	 */
	void (*maadap_notif_changed)(enum bt_snes_notifification_status maadap_notification);

		/**
	 * @brief Microphone AAD D1 params Notification callback 
	 *
	 * Indicate the CCCD descriptor status of the SNES Mic AAD D1 params characteristic.
	 * 
	 * @param[in] mig_notification Mic Input Gain Notification status (enable/disable).
	 */
	void (*maadd1p_notif_changed)(enum bt_snes_notifification_status maadd1p_notification);
};

void on_snes_connected(struct bt_conn* conn);
void on_snes_disconnected(void);

/**@brief Initialize the service.
 *
 * @details This function registers a GATT service with two characteristics,
 *          TX and RX. A remote device that is connected to this service
 *          can send data to the RX Characteristic. When the remote enables
 *          notifications, it is notified when data is sent to the TX
 *          Characteristic.
 *
 * @param[in] callbacks  Struct with function pointers to callbacks for service
 *                       events. If no callbacks are needed, this parameter can
 *                       be NULL.
 *
 * @retval 0 If initialization is successful.
 *           Otherwise, a negative value is returned.
 */
int bt_snes_init(struct bt_snes_cb *callbacks);

/**@brief Send data.
 *
 * @details This function sends data to a connected peer, or all connected
 *          peers.
 *
 * @param[in] conn Pointer to connection object, or NULL to send to all
 *                 connected peers.
 * @param[in] data Pointer to a data buffer.
 * @param[in] len  Length of the data in the buffer.
 *
 * @retval 0 If the data is sent.
 *           Otherwise, a negative value is returned.
 */
int bt_snes_cmd_send(struct bt_conn *conn, const uint8_t *data, uint16_t len);

/** @brief Update main status value.
 *
 * Update the characteristic value of the main status
 * This will send a GATT notification to all current subscribers.
 *
 *  @param status The current status.
 *
 *  @return Zero in case of success and error code in case of error.
 */
int bt_snes_update_status_cb(uint8_t status);

/** @brief Update the number of recording days.
 *
 * Update the characteristic value of the days of records
 * This will send a GATT notification to all current subscribers.
 *
 *  @param dor The current number of recording days.
 *
 *  @return Zero in case of success and error code in case of error.
 */
int bt_snes_update_days_of_records_cb(uint8_t dor);

/** @brief Update the number of recording days.
 *
 * Update the characteristic value of the device identifier.
 * This ID is add as name suffix to the BT device name
 * This will send a GATT notification to all current subscribers.
 *
 *  @param device_id The current device identifier.
 *
 *  @return Zero in case of success and error code in case of error.
 */
int bt_snes_update_device_identifier_cb(uint8_t device_id);

/** @brief Update the microphone gain.
 *
 * Update the characteristic value of the mic input gain.
 * This will send a GATT notification to all current subscribers.
 *
 *  @param input_gain The current mic input gain. the value MUST be in { 1, 2, 3, 4, 5 }.
 *
 *  @return Zero in case of success and error code in case of error.
 */
int bt_snes_update_mic_input_gain_cb(uint8_t input_gain);

/** @brief Update the microphone AAD A parameters.
 *
 * Update the characteristic values of the mic AAD A parameters.
 * This will send a GATT notification to all current subscribers.
 *
 * @param input_lpf The current Low Pass Filter value. The value MUST be in { 0x1 to 0x7 }
 * @param input_th The current Threshold value. The value MUST be in { 0x0 to 0xF }
 *
 * @return Zero in case of success and error code in case of error.
 */
int bt_snes_update_aad_a_params_cb(uint8_t input_lpf, uint8_t input_th);

/** @brief Update the microphone AAD Digital parameters.
 * 
 *  * Update the characteristic values of the mic AAD D1 parameters.
 * This will send a GATT notification to all current subscribers.
 * 
 * @param algo_sel The current Algorithm selection values. The value MUST be in { 0x0 to 0x3 }.
 * @param floor The current Floor value values. The value MUST be in { 0x00F to 0x7BC }.
 * @param rel_pulse The current Relative pulse min values. The value MUST be in { 0x000 to 0x12C }.
 * @param abs_pulse The current Absolute pulse min values. The value MUST be in { 0x000 to 0xDAC }.
 * @param rel_thr The current Relative threshold values. The value MUST be in { 0x24 to 0xFF }.
 * @param abs_thr The current Absolute threshold values. The value MUST be in { 0x00F to 0x7BC }.
 * 
 * @return Zero in case of success and error code in case of error.
 */
int bt_snes_update_aad_d1_params_cb(uint8_t algo_sel, uint16_t floor, 
                                  uint16_t rel_pulse, uint16_t abs_pulse, 
                                  uint8_t rel_thr, uint16_t abs_thr);

/**@brief Get maximum data length that can be used for @ref bt_snes_send.
 *
 * @param[in] conn Pointer to connection Object.
 *
 * @return Maximum data length.
 */
static inline uint32_t bt_snes_get_mtu(struct bt_conn *conn)
{
	/* According to 3.4.7.1 Handle Value Notification off the ATT protocol.
	 * Maximum supported notification is ATT_MTU - 3 */
	return bt_gatt_get_mtu(conn) - 3;
}

#ifdef __cplusplus
}
#endif

/**
 *@}
 */

#endif /* BT_SPEAK_NO_EVIL_SRV_H_ */
