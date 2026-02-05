/*
 * Copyright (C) Hes-so VALAIS/WALLIS, HEI Sion, Infotronics.
 * Created by Patrice Rudaz
 * All rights reserved.
 */

/* 
 * +--------------------------------------------------------
 * | Revision | Date       | Author | Brief
 * +----------+------------+--------------------------------
 * |  v2.0.0  | 06.02.2025 |  arey  | Use nRF54L15, T5848 microphone with intelligent audio level detection      
 *                                    to trigger audio saving and implement BLE proximity detection
 * |  v1.2.2  | 02.11.2023 |   rut  | Set advertisement TX power to +3dBm dynamically
 * |  v1.2.1  | 03.10.2023 |   rut  | Added System Reset when stopping recording
 * |  v1.2.0  | 31.08.2023 |   rut  | Added Dynamic mic gain & device ID
 * |  v1.1.0  | 21.08.2023 |   rut  | Added Date and Time of record’s files on SD Card
 * |  v1.0.0  | 08.08.2023 |   rut  | Added Current Time feature
 * |  v0.4.1  | 02.06.2023 |   rut  | Added option to advertise SNES UUID
 * |  v0.4.0  | 02.06.2023 |   rut  | Move from HSS to SNES, added « Nylon Wire Burning » feature
 * |  v0.3.0  | 23.05.2023 |   rut  | Added HSS BLE service
 * |  v0.3.0  | 23.05.2023 |   rut  | Added HSS BLE service
 * |  v0.2.0  | 27.04.2023 |   rut  | Adding double buffering
 * |  v0.1.0  | 20.04.2023 |   rut  | Saving every second
 * +----------+------------+--------------------------------
 */
#ifndef FIRMWARE_REVISION_H
#define FIRMWARE_REVISION_H

// firmware revision
#define FIRMWARE_REVISION                   "2.0.0-SNAPSHOT"
#define FIRMWARE_REVISION_MAJOR             2
#define FIRMWARE_REVISION_MINOR             0
#define FIRMWARE_REVISION_PATCH             0
#define FIRMWARE_REVISION_NUMBER            020000       // This is the firmware revision expressed as an int. Example: 1.2.3 -> 010203

#endif // FIRMWARE_REVISION_H

