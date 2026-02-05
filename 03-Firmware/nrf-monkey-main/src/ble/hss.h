/*
 * Copyright (c) 2023 HESSO-VS, HEI Sion
 */

#ifndef BT_HEI_SYNC_SRV_H_
#define BT_HEI_SYNC_SRV_H_

/**
 * @file
 * @defgroup bt_hei_sync HEI Sync (HSS) GATT Service
 * @{
 * @brief HEI Sync (HSS) GATT Service API.
 */

#include <zephyr/types.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief UUID of the HSS Service. 
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
#define BT_UUID_HSS_VAL \
	BT_UUID_128_ENCODE(0x00000101, 0x4865, 0x7673, 0x025A, 0x4845532D534F)

/** @brief UUID of the TX Characteristic. **/
#define BT_UUID_HSS_TX_VAL \
	BT_UUID_128_ENCODE(0x00000102, 0x4865, 0x7673, 0x025A, 0x4845532D534F)

/** @brief UUID of the RX Characteristic. **/
#define BT_UUID_HSS_RX_VAL \
	BT_UUID_128_ENCODE(0x00000103, 0x4865, 0x7673, 0x025A, 0x4845532D534F)

/** @brief UUID of the Status Characteristic. **/
#define BT_UUID_HSS_STATUS_VAL \
	BT_UUID_128_ENCODE(0x00000104, 0x4865, 0x7673, 0x025A, 0x4845532D534F)

/** @brief UUID of the Status Characteristic. **/
#define BT_UUID_HSS_DOR_VAL \
	BT_UUID_128_ENCODE(0x00000105, 0x4865, 0x7673, 0x025A, 0x4845532D534F)


#define BT_UUID_HSS_SERVICE			BT_UUID_DECLARE_128(BT_UUID_HSS_VAL)
#define BT_UUID_HSS_RX      		BT_UUID_DECLARE_128(BT_UUID_HSS_RX_VAL)
#define BT_CUD_HSS_RX				"RX"
#define BT_UUID_HSS_TX      		BT_UUID_DECLARE_128(BT_UUID_HSS_TX_VAL)
#define BT_CUD_HSS_TX				"TX"
#define BT_UUID_HSS_STATUS  		BT_UUID_DECLARE_128(BT_UUID_HSS_STATUS_VAL)
#define BT_CUD_HSS_STATUS			"Status"
#define BT_UUID_HSS_DAYS_OF_RECORDS	BT_UUID_DECLARE_128(BT_UUID_HSS_DOR_VAL)
#define BT_CUD_HSS_DAYS_OF_RECORDS	"Nbr of Days recording"

/** @brief HSS send status. */
enum bt_hss_send_status {
	/** Send notification enabled. */
	BT_HSS_SEND_STATUS_ENABLED,
	/** Send notification disabled. */
	BT_HSS_SEND_STATUS_DISABLED,
};

/** @brief Pointers to the callback functions for service events. */
struct bt_hss_cb {
	/** @brief Data received callback.
	 *
	 * The data has been received as a write request on the HSS RX
	 * Characteristic.
	 *
	 * @param[in] conn  Pointer to connection object that has received data.
	 * @param[in] data  Received data.
	 * @param[in] len   Length of received data.
	 */
	void (*received)(struct bt_conn *conn, const uint8_t *const data, uint16_t len);

	/** @brief Data sent callback.
	 *
	 * The data has been sent as a notification and written on the HSS TX
	 * Characteristic.
	 *
	 * @param[in] conn Pointer to connection object, or NULL if sent to all
	 *                 connected peers.
	 */
	void (*sent)(struct bt_conn *conn);

	/** @brief Send state callback.
	 *
	 * Indicate the CCCD descriptor status of the HSS TX characteristic.
	 *
	 * @param[in] status Send notification status.
	 */
	void (*send_enabled)(enum bt_hss_send_status status);

};

void on_hss_connected(struct bt_conn* conn);
void on_hss_disconnected(void);

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
int bt_hss_init(struct bt_hss_cb *callbacks);

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
int bt_hss_send(struct bt_conn *conn, const uint8_t *data, uint16_t len);

/** @brief Update main status value.
 *
 * Update the characteristic value of the main status
 * This will send a GATT notification to all current subscribers.
 *
 *  @param status The current status.
 *
 *  @return Zero in case of success and error code in case of error.
 */
int bt_hss_update_status_cb(uint8_t status);

/** @brief Update the number of recording days.
 *
 * Update the characteristic value of the days of records
 * This will send a GATT notification to all current subscribers.
 *
 *  @param dor The current number of recording days.
 *
 *  @return Zero in case of success and error code in case of error.
 */
int bt_hss_update_days_of_records_cb(uint8_t dor);

/**@brief Get maximum data length that can be used for @ref bt_hss_send.
 *
 * @param[in] conn Pointer to connection Object.
 *
 * @return Maximum data length.
 */
static inline uint32_t bt_hss_get_mtu(struct bt_conn *conn)
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

#endif /* BT_HEI_SYNC_SRV_H_ */
