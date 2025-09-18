//
// Copyright (c) 2025 HESSO-VS, HEI Sion
//

#ifndef _BTN_HAL_DEFINE_H_
#define _BTN_HAL_DEFINE_H_

#include <stdbool.h>

bool btn_hal_buttons_init(void);

bool btn0_has_been_pressed(void);
bool btn1_has_been_pressed(void);
bool btn2_has_been_pressed(void);
bool btn3_has_been_pressed(void);

#endif // #ifndef _BTN_HAL_DEFINE_H_
