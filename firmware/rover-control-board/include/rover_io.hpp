#pragma once

// drivers
#include <driver/gpio.h>
#include <driver/uart.h>

#define ROVER_A1_PIN  GPIO_NUM_1
#define ROVER_A2_PIN  GPIO_NUM_2
#define ROVER_A3_PIN  GPIO_NUM_3
#define ROVER_A4_PIN  GPIO_NUM_4
#define ROVER_A5_PIN  GPIO_NUM_5
#define ROVER_A6_PIN  GPIO_NUM_6
#define ROVER_A7_PIN  GPIO_NUM_7
#define ROVER_A8_PIN  GPIO_NUM_8
#define ROVER_A9_PIN  GPIO_NUM_9
#define ROVER_A10_PIN GPIO_NUM_10

void ioInitUart(uart_port_t uart, int baudRate, int pinTx = -1, int pinRx = -1);
void ioInitNvs();
void ioInitFiles();
void ioStopFiles();

void ioConfigInput(gpio_num_t gpio, gpio_pull_mode_t pull = GPIO_PULLDOWN_ONLY);
void ioConfigOutput(gpio_num_t gpio, bool openDrain = false);