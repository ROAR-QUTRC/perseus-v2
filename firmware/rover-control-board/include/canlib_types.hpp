#pragma once

#include <cstdint>
#include <functional>
#include <map>
#include <string>

#include "canlib_constants.hpp"

#define CANLIB_GET_FILTER_MASK(_filterlevel) ((1UL << (uint8_t)(_filterlevel)) - 1)

// --- ENUMS ---
enum canlib_error
{
    CANLIB_OK,
    CANLIB_ERROR_GENERIC,
    CANLIB_ERROR_INITIALISATION_FAILED,
    CANLIB_ERROR_INVALID_LENGTH,
    CANLIB_ERROR_LENGTH_MISMATCH,
    CANLIB_ERROR_DRIVER_NOT_INSTALLED,
    CANLIB_ERROR_DRIVER_NOT_RUNNING,
    CANLIB_ERROR_DRIVER_INVALID_STATE,
    CANLIB_ERROR_INVALID_INTERVAL,
    CANLIB_ERROR_NO_PARAMETER,
    CANLIB_ERROR_DUPLICATE_PARAMETER,
    CANLIB_ERROR_NOT_INITIALISED,
    CANLIB_ERROR_TIMEOUT,
};

enum canlib_address_filter_level : uint8_t
{
    CANLIB_FILTER_PARAM     = 0,
    CANLIB_FILTER_GROUP     = 4,
    CANLIB_FILTER_DEVICE    = 12,
    CANLIB_FILTER_SUBSYSTEM = 20,
    CANLIB_FILTER_SYSTEM    = 24,
    CANLIB_FILTER_NONE      = 29,
};

enum canlib_system : uint8_t
{
    CANLIB_SYSTEM_RESERVED   = 0x00,
    CANLIB_SYSTEM_POWER      = 0x01,
    CANLIB_SYSTEM_DRIVE      = 0x02,
    CANLIB_SYSTEM_EXCAVATION = 0x03,
    CANLIB_SYSTEM_SCIENCE    = 0x04,
    CANLIB_SYSTEM_DEBUG      = 0x1F,
};

// these parameter groups are reserved for ALL devices
enum canlib_reserved_param_groups : uint8_t
{
    CANLIB_GROUP_COMMON = 0x00,  // information and controls for the MCU on each device
    CANLIB_GROUP_DEBUG  = 0xFF,
};

// --- TYPEDEFS & STRUCTS ---
struct canlib_address
{
    uint8_t system : CANLIB_SYSTEM_ADDRESS_BITS       = 0;
    uint8_t subsystem : CANLIB_SUBSYSTEM_ADDRESS_BITS = 0;
    uint8_t device : CANLIB_DEVICE_ADDRESS_BITS       = 0;
    uint8_t group : CANLIB_GROUP_ADDRESS_BITS         = 0;
    uint8_t parameter : CANLIB_PARAM_ADDRESS_BITS     = 0;
    canlib_address_filter_level filter_bits           = CANLIB_FILTER_DEVICE;

    static canlib_address fromUint32(uint32_t id)
    {
        canlib_address addr = {};
        // since bitfields are defined for each value, we only have to bitshift the value
        // no binary-and required!
        addr.system    = (id >> CANLIB_SYSTEM_ADDRESS_POS);
        addr.subsystem = (id >> CANLIB_SUBSYSTEM_ADDRESS_POS);
        addr.device    = (id >> CANLIB_DEVICE_ADDRESS_POS);
        addr.group     = (id >> CANLIB_GROUP_ADDRESS_POS);
        addr.parameter = (id >> CANLIB_PARAM_ADDRESS_POS);

        return addr;
    }
    explicit operator uint32_t() const
    {
        uint32_t address = 0;
        address |= CANLIB_SYSTEM_TO_BITS(system);
        address |= CANLIB_SUBSYSTEM_TO_BITS(subsystem);
        address |= CANLIB_DEVICE_TO_BITS(device);
        address |= CANLIB_GROUP_TO_BITS(group);
        address |= CANLIB_PARAM_TO_BITS(parameter);
        return address;
    }
    bool operator==(const canlib_address& rhs) const
    {
        return ((uint32_t)(*this)) == ((uint32_t)rhs);
    }
};

typedef std::function<void(void)> canlib_handler_t;
// function is passed the address of the data relevant to it
typedef std::function<void(canlib_address)> canlib_data_handler_t;

struct canlib_data
{
    uint8_t data_arr[CANLIB_MAX_PACKET_LENGTH] = {0};
    uint8_t data_len                           = 0;
};

struct canlib_parameter_description
{
    canlib_address address = {};

    bool writable     = true;
    bool broadcast    = false;
    uint32_t interval = 0;

    canlib_data_handler_t timeout_handler          = CANLIB_NO_HANDLER;
    canlib_data_handler_t timeout_recovery_handler = CANLIB_NO_HANDLER;
    canlib_data_handler_t data_change_handler      = CANLIB_NO_HANDLER;
};

struct canlib_packet
{
    canlib_address address;
    canlib_data data;
    bool data_request = false;
};

class CanlibParameterGroup
{
public:
    CanlibParameterGroup() {}
    virtual ~CanlibParameterGroup();

    canlib_error getParameterAddress(const std::string name, canlib_address& out);

    canlib_error addParameter(const std::string name, const canlib_parameter_description description,
                              const canlib_data initialData, const canlib_data dataSafeState = {});

    template <typename T>
    canlib_error addParameter(const std::string name, const canlib_parameter_description description, const T initialData)
    {
        canlib_data tmpData;
        if (canlib_error err = canlibEncodeData(initialData, tmpData); err != CANLIB_OK)
            return err;

        return addParameter(name, description, tmpData);
    }
    template <typename T>
    canlib_error addParameter(const std::string name, canlib_parameter_description description,
                              const T initialData, const T dataSafeState)
    {
        canlib_data tmpData;
        if (canlib_error err = canlibEncodeData(initialData, tmpData); err != CANLIB_OK)
            return err;

        canlib_data tmpSafeState;
        if (canlib_error err = canlibEncodeData(initialData, tmpSafeState); err != CANLIB_OK)
            return err;

        return addParameter(name, description, tmpData, tmpSafeState);
    }

    template <typename T>
    canlib_error getParameter(const std::string name, T& out)
    {
        if (!_savedParams.contains(name))
            return CANLIB_ERROR_NO_PARAMETER;
        const canlib_address addr = _savedParams[name];

        return canlibGetParameter(addr, out);
    }
    template <typename T>
    canlib_error setParameter(const std::string name, const T data)
    {
        if (!_savedParams.contains(name))
            return CANLIB_ERROR_NO_PARAMETER;
        const canlib_address addr = _savedParams[name];

        return canlibSetParameter(addr, data);
    }

private:
    // TODO: Rule of Three / Five / Seven, or swap?
    // https://stackoverflow.com/questions/6077143/disable-copy-constructor
    // https://stackoverflow.com/questions/3106110/what-is-move-semantics
    // https://stackoverflow.com/questions/3279543/what-is-the-copy-and-swap-idiom
    // disable copy constructor and copy assignments
    CanlibParameterGroup(const CanlibParameterGroup&)  = delete;
    CanlibParameterGroup(const CanlibParameterGroup&&) = delete;

    std::map<std::string, canlib_address> _savedParams;
};