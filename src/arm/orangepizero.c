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
#include "arm/orangepizero.h"
#include "common.h"

#define DT_BASE "/proc/device-tree"

#define PLATFORM_NAME_ORANGE_PI_ZERO "Xunlong Orange Pi Zero"

#define MAX_SIZE 64

const char* orangepizero_serialdev[MRAA_ORANGEPIZERO_UART_COUNT] = { "/dev/ttyS0", "/dev/ttyS1", "/dev/ttyS2" };

void
mraa_orangepizero_pininfo(mraa_board_t* board, int index, int sysfs_pin, mraa_pincapabilities_t pincapabilities_t, char* fmt, ...)
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
mraa_orangepizero()
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
    b->phy_pin_count = MRAA_ORANGEPIZERO_PIN_COUNT + 1;

    if (mraa_file_exist(DT_BASE "/model")) {
        // We are on a modern kernel, great!!!!
        if (mraa_file_contains(DT_BASE "/model", PLATFORM_NAME_ORANGE_PI_ZERO)  ) {
            b->platform_name = PLATFORM_NAME_ORANGE_PI_ZERO;
            b->uart_dev[0].device_path = (char*) orangepizero_serialdev[0];
            b->uart_dev[1].device_path = (char*) orangepizero_serialdev[1];
            b->uart_dev[2].device_path = (char*) orangepizero_serialdev[2];
        }
    }

    // UART
    b->uart_dev_count = MRAA_ORANGEPIZERO_UART_COUNT;
    b->def_uart_dev = 0;
    b->uart_dev[0].index = 0;
    b->uart_dev[1].index = 1;
    b->uart_dev[2].index = 2;

    // I2C
    if (strncmp(b->platform_name, PLATFORM_NAME_ORANGE_PI_ZERO, MAX_SIZE) == 0) {
        b->i2c_bus_count = MRAA_ORANGEPIZERO_I2C_COUNT;
        b->def_i2c_bus = 0;
        b->i2c_bus[0].bus_id = 0;
        b->i2c_bus[1].bus_id = 1;
    }

    // SPI
    b->spi_bus_count = MRAA_ORANGEPIZERO_SPI_COUNT;
    b->def_spi_bus = 0;
    b->spi_bus[0].bus_id = 1;

    // PWM
    b->pwm_dev_count = MRAA_ORANGEPIZERO_PWM_COUNT;
    b->pwm_default_period = 500;
    b->pwm_max_period = 2147483;
    b->pwm_min_period = 1;

    b->pins = (mraa_pininfo_t*) malloc(sizeof(mraa_pininfo_t) * b->phy_pin_count);
    if (b->pins == NULL) {
        free(b->adv_func);
        free(b);
        return NULL;
    }

    b->pins[7].pwm.parent_id = 1;
    b->pins[7].pwm.mux_total = 0;
    b->pins[7].pwm.pinmap = 0;

    b->aio_count = 0;

    mraa_orangepizero_pininfo(b, 0,   -1, (mraa_pincapabilities_t){0,0,0,0,0,0,0,0}, "INVALID");
    mraa_orangepizero_pininfo(b, 1,   -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "3V3");
    mraa_orangepizero_pininfo(b, 2,   -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "5V");
    mraa_orangepizero_pininfo(b, 3,   12, (mraa_pincapabilities_t){1,1,0,0,0,1,0,0}, "I2C0_SDA,GPIOA12");
    mraa_orangepizero_pininfo(b, 4,   -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "5V");
    mraa_orangepizero_pininfo(b, 5,   11, (mraa_pincapabilities_t){1,1,0,0,0,1,0,0}, "I2C0_SCL,GPIOA11");
    mraa_orangepizero_pininfo(b, 6,   -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "GND");
    mraa_orangepizero_pininfo(b, 7,    6, (mraa_pincapabilities_t){1,1,1,0,0,0,0,0}, "PWM1,GPIOA06");
    mraa_orangepizero_pininfo(b, 8,  198, (mraa_pincapabilities_t){1,1,0,0,0,0,0,1}, "UART1_TX,GPIOG06");
    mraa_orangepizero_pininfo(b, 9,   -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "GND");
    mraa_orangepizero_pininfo(b, 10, 199, (mraa_pincapabilities_t){1,1,0,0,0,0,0,1}, "UART1_RX,GPIOG07");
    mraa_orangepizero_pininfo(b, 11,   1, (mraa_pincapabilities_t){1,1,0,0,0,0,0,1}, "UART2_RX,GPIOA01");
    mraa_orangepizero_pininfo(b, 12,   7, (mraa_pincapabilities_t){1,1,0,0,0,0,0,0}, "SIM_CLK,PA_EINT7,GPIOA07");
    mraa_orangepizero_pininfo(b, 13,   0, (mraa_pincapabilities_t){1,1,0,0,0,0,0,1}, "UART2_TX,GPIOA00");
    mraa_orangepizero_pininfo(b, 14,  -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "GND");
    mraa_orangepizero_pininfo(b, 15,   3, (mraa_pincapabilities_t){1,1,0,0,0,0,0,1}, "UART2_CTS,GPIOA03");
    mraa_orangepizero_pininfo(b, 16,  19, (mraa_pincapabilities_t){1,1,0,0,0,1,0,0}, "I2C1_SDA,GPIOA19");
    mraa_orangepizero_pininfo(b, 17,  -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "3V3");
    mraa_orangepizero_pininfo(b, 18,  18, (mraa_pincapabilities_t){1,1,0,0,0,1,0,0}, "I2C1_SCK,GPIOA18");
    mraa_orangepizero_pininfo(b, 19,  15, (mraa_pincapabilities_t){1,1,0,0,1,0,0,0}, "SPI1_MOSI,GPIOA15");
    mraa_orangepizero_pininfo(b, 20,  -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "GND");
    mraa_orangepizero_pininfo(b, 21,  16, (mraa_pincapabilities_t){1,1,0,0,1,0,0,0}, "SPI1_MISO,GPIOA16");
    mraa_orangepizero_pininfo(b, 22,   2, (mraa_pincapabilities_t){1,1,0,0,0,0,0,1}, "UART2_RTS,GPIOA02");
    mraa_orangepizero_pininfo(b, 23,  14, (mraa_pincapabilities_t){1,1,0,0,1,0,0,0}, "SPI1_CLK,GPIOA14");
    mraa_orangepizero_pininfo(b, 24,  13, (mraa_pincapabilities_t){1,1,0,0,1,0,0,0}, "SPI1_CS,GPIOA13");
    mraa_orangepizero_pininfo(b, 25,  -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "GND");
    mraa_orangepizero_pininfo(b, 26,  10, (mraa_pincapabilities_t){1,1,0,0,0,0,0,0}, "SIM_DET,PA_EINT10,GPIOA10");

    return b;
}
