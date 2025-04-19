#pragma once

#include <driver/gpio.h>

/// @brief Board Support Package - contains board-specific constants and configurations
namespace bsp
{
    constexpr gpio_num_t CAN_TX_PIN = GPIO_NUM_0;
    constexpr gpio_num_t CAN_RX_PIN = GPIO_NUM_11;

    constexpr gpio_num_t I2C_SDA = GPIO_NUM_35;
    constexpr gpio_num_t I2C_SCL = GPIO_NUM_36;

    void initI2C();
}