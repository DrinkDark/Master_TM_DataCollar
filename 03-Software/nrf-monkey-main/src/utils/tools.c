/*
 * Copyright (c) 2023, HES-SO Valais-Wallis, HEI, Sion
 */

#include "tools.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>


bool is_digit(char c)
{
    return (c == '0' || c == '1' || c == '2' || c == '3' || c == '4' || c == '5' || c == '6' || c == '7' || c == '8' || c == '9');
}

// int uint32_t_to_int(uint32_t value)
// {
//     int result = 0;
//     if (value > 0x7fffffff) {
//         result = 0x7fffffff - ((int) (value & 0xffffffff));
//     } else {
//         result = (int) (value & 0xffffffff);
//     }
// }
