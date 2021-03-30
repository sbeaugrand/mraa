/*
 * Author: Sebastien Beaugrand
 * Copyright (c) 2021 Sebastien Beaugrand
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "mraa_internal.h"

#define MRAA_ORANGEPIZERO_I2C_COUNT  2
#define MRAA_ORANGEPIZERO_SPI_COUNT  1
#define MRAA_ORANGEPIZERO_UART_COUNT 3
#define MRAA_ORANGEPIZERO_PWM_COUNT  1
#define MRAA_ORANGEPIZERO_PIN_COUNT  26

mraa_board_t *
        mraa_orangepizero();

#ifdef __cplusplus
}
#endif
