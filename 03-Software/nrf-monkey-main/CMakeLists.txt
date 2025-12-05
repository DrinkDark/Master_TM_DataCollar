# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

# set(LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/packages/lib)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(nrf_monkey)

# defining some environmental variables
if(CMAKE_HOST_SYSTEM MATCHES "Linux")
    message("==========  Linux   =========")
elseif(CMAKE_HOST_SYSTEM MATCHES "Darwin")
    message("==========  APPLE  =========")
elseif(CMAKE_HOST_SYSTEM MATCHES "Windows")
    message("========== Windows =========")
endif()

# Source code
target_sources(app PRIVATE src/main.c)
target_sources(app PRIVATE src/hal/gpio_hal.c)
target_sources(app PRIVATE src/utils/tools.c)
target_sources(app PRIVATE src/storage/flash.c)
target_sources(app PRIVATE src/ble/sig_dis.c)
target_sources(app PRIVATE src/microphone/t5848.c)
# target_sources(app PRIVATE src/storage/nvs_flash.c)
target_sources_ifdef(CONFIG_FILE_SYSTEM app PRIVATE src/fatfs/sdcard.c)
target_sources_ifdef(CONFIG_I2S app PRIVATE src/audio/recorder.c)
target_sources_ifdef(CONFIG_BT app PRIVATE src/ble/ble_handler.c)
target_sources_ifdef(CONFIG_BT_SNES_SRV app PRIVATE src/ble/snes.c)
target_sources_ifdef(CONFIG_BT_CTS_CLIENT app PRIVATE src/ble/sig_cts.c)

