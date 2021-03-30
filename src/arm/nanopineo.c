/*
 * Author: Sebastien Beaugrand
 * Copyright (c) 2021 Sebastien Beaugrand
 *
 * SPDX-License-Identifier: MIT
 */

#include <mraa/common.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include "arm/nanopineo.h"
#include "common.h"

#define DT_BASE "/proc/device-tree"

#define PLATFORM_NAME_NANO_PI_NEO "FriendlyARM NanoPi NEO"

#define MAX_SIZE 64

const char* nanopineo_serialdev[MRAA_NANOPINEO_UART_COUNT] = { "/dev/ttyS0", "/dev/ttyS1", "/dev/ttyS2" };

void
mraa_nanopineo_pininfo(mraa_board_t* board, int index, int sysfs_pin, mraa_pincapabilities_t pincapabilities_t, char* fmt, ...)
{
    va_list arg_ptr;

    if (index > board->phy_pin_count)
        return;

    mraa_pininfo_t* pininfo = &board->pins[index];
    va_start(arg_ptr, fmt);
    vsnprintf(pininfo->name, MRAA_PIN_NAME_SIZE, fmt, arg_ptr);

    if (pincapabilities_t.gpio == 1) {
        va_arg(arg_ptr, int);
        pininfo->gpio.gpio_chip = va_arg(arg_ptr, int);
        pininfo->gpio.gpio_line = va_arg(arg_ptr, int);
    }

    pininfo->capabilities = pincapabilities_t;

    va_end(arg_ptr);
    pininfo->gpio.pinmap = sysfs_pin;
    pininfo->gpio.mux_total = 0;
}

mraa_board_t*
mraa_nanopineo()
{
    mraa_board_t* b = (mraa_board_t*) calloc(1, sizeof(mraa_board_t));
    if (b == NULL) {
        return NULL;
    }

    b->adv_func = (mraa_adv_func_t*) calloc(1, sizeof(mraa_adv_func_t));
    if (b->adv_func == NULL) {
        free(b);
        return NULL;
    }

    // pin mux for buses are setup by default by kernel so tell mraa to ignore them
    b->no_bus_mux = 1;
    b->phy_pin_count = MRAA_NANOPINEO_PIN_COUNT + 1;

    if (mraa_file_exist(DT_BASE "/model")) {
        // We are on a modern kernel, great!!!!
        if (mraa_file_contains(DT_BASE "/model", PLATFORM_NAME_NANO_PI_NEO)  ) {
            b->platform_name = PLATFORM_NAME_NANO_PI_NEO;
            b->uart_dev[0].device_path = (char*) nanopineo_serialdev[0];
            b->uart_dev[1].device_path = (char*) nanopineo_serialdev[1];
            b->uart_dev[2].device_path = (char*) nanopineo_serialdev[2];
        }
    }

    // UART
    b->uart_dev_count = MRAA_NANOPINEO_UART_COUNT;
    b->def_uart_dev = 0;
    b->uart_dev[0].index = 0;
    b->uart_dev[1].index = 1;
    b->uart_dev[2].index = 2;

    // I2C
    if (strncmp(b->platform_name, PLATFORM_NAME_NANO_PI_NEO, MAX_SIZE) == 0) {
        b->i2c_bus_count = MRAA_NANOPINEO_I2C_COUNT;
        b->def_i2c_bus = 0;
        b->i2c_bus[0].bus_id = 0;
    }

    // SPI
    b->spi_bus_count = MRAA_NANOPINEO_SPI_COUNT;
    b->def_spi_bus = 0;
    b->spi_bus[0].bus_id = 0;

    // PWM
    b->pwm_dev_count = 0;

    b->pins = (mraa_pininfo_t*) malloc(sizeof(mraa_pininfo_t) * b->phy_pin_count);
    if (b->pins == NULL) {
        free(b->adv_func);
        free(b);
        return NULL;
    }

    b->aio_count = 0;

    mraa_nanopineo_pininfo(b, 0,   -1, (mraa_pincapabilities_t){0,0,0,0,0,0,0,0}, "INVALID");
    mraa_nanopineo_pininfo(b, 1,   -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "3V3");
    mraa_nanopineo_pininfo(b, 2,   -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "5V");
    mraa_nanopineo_pininfo(b, 3,   -1, (mraa_pincapabilities_t){1,0,0,0,0,1,0,0}, "I2C0_SDA");
    mraa_nanopineo_pininfo(b, 4,   -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "5V");
    mraa_nanopineo_pininfo(b, 5,   -1, (mraa_pincapabilities_t){1,0,0,0,0,1,0,0}, "I2C0_SCL");
    mraa_nanopineo_pininfo(b, 6,   -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "GND");
    mraa_nanopineo_pininfo(b, 7,  203, (mraa_pincapabilities_t){1,1,0,0,0,0,0,0}, "GPIOG11");
    mraa_nanopineo_pininfo(b, 8,  198, (mraa_pincapabilities_t){1,1,0,0,0,0,0,1}, "UART1_TX,GPIOG6");
    mraa_nanopineo_pininfo(b, 9,   -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "GND");
    mraa_nanopineo_pininfo(b, 10, 199, (mraa_pincapabilities_t){1,1,0,0,0,0,0,1}, "UART1_RX,GPIOG7");
    mraa_nanopineo_pininfo(b, 11,   0, (mraa_pincapabilities_t){1,1,0,0,0,0,0,1}, "UART2_TX,GPIOA0");
    mraa_nanopineo_pininfo(b, 12,   6, (mraa_pincapabilities_t){1,1,0,0,0,0,0,0}, "GPIOA6");
    mraa_nanopineo_pininfo(b, 13,   2, (mraa_pincapabilities_t){1,1,0,0,0,0,0,1}, "UART2_RTS,GPIOA2");
    mraa_nanopineo_pininfo(b, 14,  -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "GND");
    mraa_nanopineo_pininfo(b, 15,   3, (mraa_pincapabilities_t){1,1,0,0,0,0,0,1}, "UART2_CTS,GPIOA3");
    mraa_nanopineo_pininfo(b, 16, 200, (mraa_pincapabilities_t){1,1,0,0,0,0,0,1}, "UART1_RTS,GPIOG8");
    mraa_nanopineo_pininfo(b, 17,  -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "3V3");
    mraa_nanopineo_pininfo(b, 18, 201, (mraa_pincapabilities_t){1,1,0,0,0,0,0,1}, "UART1_CTS,GPIOG9");
    mraa_nanopineo_pininfo(b, 19,  64, (mraa_pincapabilities_t){1,1,0,0,1,0,0,0}, "SPI0_MOSI,GPIOC0");
    mraa_nanopineo_pininfo(b, 20,  -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "GND");
    mraa_nanopineo_pininfo(b, 21,  65, (mraa_pincapabilities_t){1,1,0,0,1,0,0,0}, "SPI0_MISO,GPIOC1");
    mraa_nanopineo_pininfo(b, 22,   1, (mraa_pincapabilities_t){1,1,0,0,0,0,0,1}, "UART2_RX,GPIOA1");
    mraa_nanopineo_pininfo(b, 23,  66, (mraa_pincapabilities_t){1,1,0,0,1,0,0,0}, "SPI0_CLK,GPIOC2");
    mraa_nanopineo_pininfo(b, 24,  67, (mraa_pincapabilities_t){1,1,0,0,1,0,0,0}, "SPI0_CS,GPIOC3");

    return b;
}
