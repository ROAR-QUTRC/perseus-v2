#pragma once

/**
 * @file st3215_servo.hpp
 * @brief Main include file for the ST3215 servo library
 * @details This library provides a C++20 interface for controlling ST3215 serial servos
 *          using the Feetech STS protocol over a serial connection via boost::asio.
 *          Supports both single servo control and synchronized multi-servo operations.
 */

#include "st3215_servo/core.hpp"
#include "st3215_servo/memory_tables.hpp"
#include "st3215_servo/packet.hpp"
#include "st3215_servo/servo.hpp"
#include "st3215_servo/servo_manager.hpp"

/**
 * @namespace st3215
 * @brief Contains all functionality for interfacing with ST3215 servos
 *
 * This namespace provides a complete interface for controlling ST3215 servos including:
 * - Single servo control via the Servo class
 * - Multi-servo management via the ServoManager class
 * - Low-level packet construction and parsing
 * - Memory map definitions and constants
 *
 * The library uses boost::asio for serial communication and supports
 * modern C++20 features for improved safety and convenience.
 */
namespace st3215
{
    // Version information
    constexpr char VERSION[] = "0.1.0";
}  // namespace st3215