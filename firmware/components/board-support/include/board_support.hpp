#pragma once

#include <driver/gpio.h>

#include <tuple>

/// @brief Board Support Package - contains board-specific constants and configurations
namespace bsp
{
    /**
     * @brief An ordered pair of GPIO pins.
     *
     * When used for a CAN bus or UART, first pin is for TX, and the second is RX.
     * When used for a motor driver, first pin is for A, and the second is for B.
     */
    typedef std::pair<gpio_num_t, gpio_num_t> pin_pair_t;

    constexpr gpio_num_t CAN_TX_PIN = GPIO_NUM_0;
    constexpr gpio_num_t CAN_RX_PIN = GPIO_NUM_11;

    constexpr gpio_num_t I2C_SDA = GPIO_NUM_35;
    constexpr gpio_num_t I2C_SCL = GPIO_NUM_36;

    constexpr gpio_num_t A1 = GPIO_NUM_1;
    constexpr gpio_num_t A2 = GPIO_NUM_2;
    constexpr gpio_num_t A3 = GPIO_NUM_3;
    constexpr gpio_num_t A4 = GPIO_NUM_4;
    constexpr gpio_num_t A5 = GPIO_NUM_5;
    constexpr gpio_num_t A6 = GPIO_NUM_6;
    constexpr gpio_num_t A7 = GPIO_NUM_7;
    constexpr gpio_num_t A8 = GPIO_NUM_8;
    constexpr gpio_num_t A9 = GPIO_NUM_9;
    constexpr gpio_num_t A10 = GPIO_NUM_10;

    void initI2C();
}