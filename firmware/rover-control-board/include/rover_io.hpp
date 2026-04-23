#pragma once

// drivers
#include <driver/gpio.h>
#include <driver/uart.h>

enum struct ROVER_PIN
{
    A1 = GPIO_NUM_1,
    A2 = GPIO_NUM_2,
    A3 = GPIO_NUM_3,
    A4 = GPIO_NUM_4,
    A5 = GPIO_NUM_5,
    A6 = GPIO_NUM_6,
    A7 = GPIO_NUM_7,
    A8 = GPIO_NUM_8,
    A9 = GPIO_NUM_9,
    A10 = GPIO_NUM_10,
};

void initialize_uart(uart_port_t uart, int baudRate, int pinTx = -1, int pinRx = -1);
void initialize_nvs();

void gpio_set_input(gpio_num_t gpio, gpio_pull_mode_t pull = GPIO_PULLDOWN_ONLY);
void gpio_set_output(gpio_num_t gpio, bool openDrain = false);