#pragma once

// standard libraries
#include <cstring>

// core
#include <driver/gpio.h>

// libraries
#include <rover_log.hpp>

// rest of the library
#include "canlib_common.hpp"
#include "canlib_types.hpp"

#define CANLIB_DEFAULT_RX GPIO_NUM_11
#define CANLIB_DEFAULT_TX GPIO_NUM_0

/// @brief Initialise CANlib, and CAN bus I/O + driver
/// @param address Device address
/// @param rx RX pin
/// @param tx TX pin
/// @return CANLIB_OK or error condition
canlib_error canlibInit(const canlib_address address, const gpio_num_t rx = CANLIB_DEFAULT_RX, const gpio_num_t tx = CANLIB_DEFAULT_TX);
/// @brief Stop CAN driver
/// @return CANLIB_OK or error condition
canlib_error canlibStop();

/// @brief Get the current device address
/// @return Current device address
canlib_address canlibGetAddress();
/// @brief Set the current device address
/// @param address New device address
/// @return CANLIB_OK or error condition
canlib_error canlibSetAddress(const canlib_address address);

/// @brief Get a parameter from CANlib's registry
/// @param address Address of the parameter
/// @param out Retreived parameter
/// @return CANLIB_OK or error condition
canlib_error canlibGetData(const canlib_address address, canlib_data& out);
/// @brief Set the value of a parameter in CANlib's registry
/// @param address Address of the parameter
/// @param data New data
/// @return CANLIB_OK or error condition
canlib_error canlibSetData(const canlib_address address, const canlib_data data);

// template functions need to be implemented in header files since they're generated at compile time

/// @brief Encode data into a canlib_data struct
/// @tparam T Data type to encode
/// @param data Data to encode
/// @param out Encoded data
/// @return CANLIB_OK or error condition
template <typename T>
canlib_error canlibEncodeData(const T data, canlib_data& out)
{
    if (sizeof(data) > CANLIB_MAX_PACKET_LENGTH)
    {
        CANLIB_ERROR("Error encoding data! Data too long (%d bytes)", sizeof(data));
        return CANLIB_ERROR_INVALID_LENGTH;
    }
    memcpy(out.data_arr, &data, sizeof(data));
    out.data_len = sizeof(data);
    return CANLIB_OK;
}
/// @brief Dencode data from a canlib_data struct
/// @tparam T Data type to decode
/// @param data Data to decode
/// @param out Decoded data
/// @return CANLIB_OK or error condition
template <typename T>
canlib_error canlibDecodeData(const canlib_data data, T& out)
{
    if (data.data_len != sizeof(out))
    {
        CANLIB_ERROR("Error decoding data! Data length mismatch (%d vs %d bytes)", data.data_len, sizeof(out));
        return CANLIB_ERROR_LENGTH_MISMATCH;
    }
    memcpy(&out, data.data_arr, data.data_len);
    return CANLIB_OK;
}

/// @brief Get the value of a parameter CANlib is tracking
/// @tparam T Type of the data to get
/// @param address The address of the parameter to get
/// @param out Retreived data
/// @return CANLIB_OK or error condition
template <typename T>
canlib_error canlibGetParameter(const canlib_address address, T& out)
{
    canlib_data data;
    if (canlib_error err = canlibGetData(address, data); err != CANLIB_OK)
        return err;

    return canlibDecodeData<T>(data, out);
}
/// @brief Set the value of a parameter CANlib is tracking
/// @tparam T Type of the data to set
/// @param address The address of the parameter to set
/// @param data New parameter value
/// @return CANLIB_OK or error condition
template <typename T>
canlib_error canlibSetParameter(const canlib_address address, const T data)
{
    canlib_data tmp;
    if (canlib_error err = canlibEncodeData(data, tmp); err != CANLIB_OK)
        return err;

    return canlibSetData(address, tmp);
}

/// @brief Add a parameter to CANlib's registry
/// @param description The full description of the parameter
/// @param initialData The data to initialise the parameter with
/// @return CANLIB_OK or error condition
canlib_error canlibAddParameter(canlib_parameter_description description, const canlib_data initialData, const canlib_data dataSafeState = {});
/// @copydoc canlibAddParameter(const canlib_parameter_description, const canlib_data, const canlib_data)
/// @tparam T The type of the data to add
template <typename T>
canlib_error canlibAddParameter(const canlib_parameter_description description, const T initialData)
{
    canlib_data tmpData;
    if (canlib_error err = canlibEncodeData(initialData, tmpData); err != CANLIB_OK)
        return err;

    return canlibAddParameter(description, tmpData);
}
/// @copydoc canlibAddParameter(const canlib_parameter_description, const T)
/// @param dataSafeState The safe state for the data to be reverted to on timeout
template <typename T>
canlib_error canlibAddParameter(canlib_parameter_description description, const T initialData, const T dataSafeState)
{
    canlib_data tmpData;
    if (canlib_error err = canlibEncodeData(initialData, tmpData); err != CANLIB_OK)
        return err;

    canlib_data tmpSafeState;
    if (canlib_error err = canlibEncodeData(initialData, tmpSafeState); err != CANLIB_OK)
        return err;

    return canlibAddParameter(description, tmpData, tmpSafeState);
}
/// @brief Removes a parameter from CANlib's registry
/// @param address Address of the parameter
/// @return CANLIB_OK or error condition
canlib_error canlibRemoveParameter(canlib_address address);

/// @brief Transmit a data packet over CAN bus
/// @param packet Packet to transmit
/// @return CANLIB_OK or error condition
canlib_error canlibTransmit(canlib_packet packet, bool useDeviceId = true);
/// @brief Handle all of CANlib's variable tracking, and RX/TX events
/// @return CANLIB_OK or error condition
canlib_error canlibHandle();