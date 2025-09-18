# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(sd_card_access_nrf54)

# defining some environmental variables
if(CMAKE_HOST_SYSTEM MATCHES "Linux")
    message("===========  Linux  ==========")
elseif(CMAKE_HOST_SYSTEM MATCHES "Darwin")
    message("===========  APPLE  ==========")
elseif(CMAKE_HOST_SYSTEM MATCHES "Windows")
    message("=========== Windows ==========")
endif()

# Source code
target_sources(app PRIVATE src/main.c)
target_sources(app PRIVATE src/hal/gpio_hal.c)
target_sources(app PRIVATE src/hal/btn_hal.c)

target_sources_ifdef(CONFIG_FILE_SYSTEM app PRIVATE src/fatfs/sdcard.c)
