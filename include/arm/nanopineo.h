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

#define MRAA_NANOPINEO_I2C_COUNT  1
#define MRAA_NANOPINEO_SPI_COUNT  1
#define MRAA_NANOPINEO_UART_COUNT 3
#define MRAA_NANOPINEO_PIN_COUNT  24

mraa_board_t *
        mraa_nanopineo();

#ifdef __cplusplus
}
#endif
