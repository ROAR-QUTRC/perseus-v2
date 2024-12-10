#pragma once

#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/macros.hpp>

#define CHECK_INTERFACE_COUNT(_logger, _joint, _interface_type, _interface_name, _count)                    \
    do                                                                                                      \
    {                                                                                                       \
        if ((_joint)._interface_type.size() != (_count))                                                    \
        {                                                                                                   \
            RCLCPP_FATAL((_logger), "Joint '%s' has %zu " _interface_name " interfaces found, expected %d", \
                         (_joint).name.c_str(),                                                             \
                         (_joint)._interface_type.size(),                                                   \
                         _count);                                                                           \
            return hardware_interface::CallbackReturn::ERROR;                                               \
        }                                                                                                   \
    } while (0)

#define CHECK_INTERFACE_NAME(_logger, _joint, _interface_type, _interface_name, _interface_index, _expected_name) \
    do                                                                                                            \
    {                                                                                                             \
        if (const auto& interfaceName = (_joint)._interface_type[(_interface_index)].name;                        \
            interfaceName != (_expected_name))                                                                    \
        {                                                                                                         \
            RCLCPP_FATAL((_logger),                                                                               \
                         "Found '%s' %s interface on joint '%s', expected '%s'",                                  \
                         interfaceName.c_str(),                                                                   \
                         _interface_name,                                                                         \
                         (_joint).name.c_str(),                                                                   \
                         _expected_name);                                                                         \
            return hardware_interface::CallbackReturn::ERROR;                                                     \
        }                                                                                                         \
    } while (0)

#define CHECK_HARDWARE_PARAMETER_EXISTS(_logger, _info, _param_name)                            \
    do                                                                                          \
    {                                                                                           \
        if ((_info).hardware_parameters.find(_param_name) == (_info).hardware_parameters.end()) \
        {                                                                                       \
            RCLCPP_FATAL((_logger),                                                             \
                         "Hardware '%s' has no '%s' parameter",                                 \
                         (_info).name.c_str(),                                                  \
                         _param_name);                                                          \
            return hardware_interface::CallbackReturn::ERROR;                                   \
        }                                                                                       \
    } while (0)

#define CHECK_PARAMETER_EXISTS(_logger, _joint, _param_name)                    \
    do                                                                          \
    {                                                                           \
        if ((_joint).parameters.find(_param_name) == (_joint).parameters.end()) \
        {                                                                       \
            RCLCPP_FATAL((_logger),                                             \
                         "Joint '%s' has no '%s' parameter",                    \
                         (_joint).name.c_str(),                                 \
                         _param_name);                                          \
            return hardware_interface::CallbackReturn::ERROR;                   \
        }                                                                       \
    } while (0)
