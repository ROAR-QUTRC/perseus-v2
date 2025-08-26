#include "canlib.hpp"

// standard libraries
#include <cstring>
#include <mutex>
#include <vector>

// core
#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/timers.h>

#include "sdkconfig.h"

// drivers
#include <driver/twai.h>

// rover libs
#include <rover_log.hpp>

#define CANLIB_NO_ACK
// #define CANLIB_LOOPBACK
#ifdef CANLIB_LOOPBACK
#define CANLIB_NO_ACK
#endif
#define CANLIB_TIMEOUT_COUNT 3

using std::mutex, std::lock_guard, std::vector;

static bool txErrorPrinted = false;
static bool rxErrorPrinted = false;
static bool gpioAssigned = false;

static canlib_address currentAddress;
static gpio_num_t currentTx, currentRx;

static mutex dataMutex;

struct parameter_wrapper
{
    canlib_parameter_description description = {};
    canlib_data data = {};
    canlib_data data_safe_state = {};
    TickType_t last_packet_tick = 0;
    bool timed_out = false;
};

static vector<parameter_wrapper> broadcastParameters;
static vector<parameter_wrapper> standardParameters;

inline uint32_t getParameterSpecifier(canlib_address address)
{
    return (((uint32_t)address) & CANLIB_GET_FILTER_MASK(currentAddress.filter_bits));
}
inline uint32_t getParameterSpecifier(uint32_t address)
{
    return (address & CANLIB_GET_FILTER_MASK(currentAddress.filter_bits));
}
inline bool driverRunning()
{
    twai_status_info_t status;
    esp_err_t err = twai_get_status_info(&status);
    if (err != ESP_OK)  // driver not installed
        return false;

    return (status.state == TWAI_STATE_RUNNING);
}

parameter_wrapper& lookupParameter(uint32_t id)
{
    static parameter_wrapper error_ret = {.data = {.data_len = CANLIB_ERROR_LENGTH}};
    for (parameter_wrapper& wrapper : standardParameters)
        if ((uint32_t)wrapper.description.address == id)
            return wrapper;
    for (parameter_wrapper& wrapper : broadcastParameters)
        if ((uint32_t)wrapper.description.address == id)
            return wrapper;
    error_ret.data.data_len = CANLIB_ERROR_LENGTH;  // ensure reset to error
    return error_ret;
}
// returns true if bus not ready
bool checkAlerts()
{
    static int recovery_attempts = 0;
    // check for alerts and errors, handle bus recovery if necessary
    uint32_t alerts = 0;
    esp_err_t err = twai_read_alerts(&alerts, 0);
    if (err != ESP_OK)  // there were alerts
        return false;   // no error

    if (alerts & TWAI_ALERT_TX_FAILED)
        CANLIB_ERROR("Packet transmission failed!");

    if (alerts & TWAI_ALERT_BUS_RECOVERED)
    {
        CANLIB_INFO("CAN bus recovery complete! Starting driver.");
        twai_start();
        txErrorPrinted = false;
        rxErrorPrinted = false;
        recovery_attempts = 0;
    }

    if (alerts & TWAI_ALERT_BUS_OFF)
    {
        twai_status_info_t state;
        err = twai_get_status_info(&state);
        if ((err != ESP_OK) || (state.state != TWAI_STATE_BUS_OFF))
            return true;

        if (recovery_attempts == CONFIG_CANLIB_BUS_RECOVERY_ATTEMPTS)
        {
            CANLIB_ERROR("Bus recovery failed. Is there a hardware fault?");
            canlibStop();
            return true;
        }
        else if (recovery_attempts > CONFIG_CANLIB_BUS_RECOVERY_ATTEMPTS)
            return true;  // just do nothing
        recovery_attempts++;
        CANLIB_ERROR("Bus has entered error state. Starting recovery, attempt %d of %d.", recovery_attempts, CONFIG_CANLIB_BUS_RECOVERY_ATTEMPTS);
        twai_initiate_recovery();
        return true;
    }
    return false;
}

canlib_error canlibInit(canlib_address address, gpio_num_t rx, gpio_num_t tx)
{
    if (driverRunning())
    {
        CANLIB_WARN("Multiple initialisations attempted! Ignoring re-initialision attempt");
        return CANLIB_OK;
    }

    CANLIB_INFO("Begin CAN bus initialisation");
    CANLIB_DEBUG("TX: %d, RX: %d", (int)tx, (int)rx);

#ifndef CANLIB_NO_ACK
    // standard configs at 125kbps
    twai_general_config_t generalConfig = TWAI_GENERAL_CONFIG_DEFAULT(tx, rx, TWAI_MODE_NORMAL);
#else
    CANLIB_WARN("Configuring CAN bus in no-ACK mode for debugging");
    // loopback - not going to get an ACK
    twai_general_config_t generalConfig = TWAI_GENERAL_CONFIG_DEFAULT(tx, rx, TWAI_MODE_NO_ACK);
#endif
    generalConfig.alerts_enabled = TWAI_ALERT_BUS_OFF | TWAI_ALERT_BUS_RECOVERED | TWAI_ALERT_TX_FAILED;

#if defined(CONFIG_CANLIB_BAUD_1M)
    twai_timing_config_t timingConfig = TWAI_TIMING_CONFIG_1MBITS();
#elif defined(CONFIG_CANLIB_BAUD_500K)
    twai_timing_config_t timingConfig = TWAI_TIMING_CONFIG_500KBITS();
#elif defined(CONFIG_CANLIB_BAUD_250K)
    twai_timing_config_t timingConfig = TWAI_TIMING_CONFIG_250KBITS();
#else  // 125K
    twai_timing_config_t timingConfig = TWAI_TIMING_CONFIG_125KBITS();
#endif

    // set up acceptance filter
    const uint32_t mask = CANLIB_GET_FILTER_MASK(address.filter_bits);
    const uint32_t addr = ((uint32_t)address) & ~mask;
    CANLIB_DEBUG("CAN bus address: 0x%08X, mask: 0x%08X", addr, mask);
    twai_filter_config_t filterConfig = {
        .acceptance_code = addr << 3,  // address and filter diagrams are RIGHT SIDE MSB! WHY!!!
        .acceptance_mask = mask << 3,  // so the mask + addr need to be shifted 3 bits right...
        .single_filter = true};

    currentAddress = address;
    currentTx = tx;
    currentRx = rx;
    gpioAssigned = true;

    // initialise driver
    esp_err_t err = twai_driver_install(&generalConfig, &timingConfig, &filterConfig);
    if (err != ESP_OK)
    {
        CANLIB_ERROR("CAN bus initialisation failed!");
        return CANLIB_ERROR_INITIALISATION_FAILED;
    }

    txErrorPrinted = false;
    rxErrorPrinted = false;

    CANLIB_DEBUG("Starting CAN receiver");
    twai_start();
    CANLIB_DEBUG("CAN bus initialisation complete");
    return CANLIB_OK;
}
canlib_error canlibStop()
{
    CANLIB_INFO("Stopping CAN bus");

    twai_status_info_t status;
    esp_err_t err = twai_get_status_info(&status);
    if (err == ESP_OK)  // driver installed
    {
        if (status.state == TWAI_STATE_RUNNING)
            twai_stop();

        twai_driver_uninstall();
    }
    else
        CANLIB_WARN("Attempting to stop CAN bus, but never initialised");

    CANLIB_DEBUG("CAN bus stopped");
    return CANLIB_OK;
}

canlib_error canlibSetAddress(canlib_address newAddress)
{
    if (newAddress == currentAddress)
    {
        CANLIB_WARN("Attempting to change address, but new address matches old! Ignoring.");
        return CANLIB_OK;
    }

    if (!gpioAssigned)
    {
        CANLIB_ERROR("Attempting to re-assign address before ever initialising!");
        return CANLIB_ERROR_NOT_INITIALISED;
    }

    if (!driverRunning())
        CANLIB_WARN("Re-assigning address while driver is in an invalid state!");
    else
        canlibStop();

    return canlibInit(newAddress, currentTx, currentRx);
}
canlib_address canlibGetAddress()
{
    return currentAddress;
}

canlib_error canlibGetData(const canlib_address address, canlib_data& out)
{
    const lock_guard dataLock(dataMutex);

    parameter_wrapper tmp = lookupParameter(getParameterSpecifier(address));
    if (tmp.data.data_len == CANLIB_ERROR_LENGTH)
        return CANLIB_ERROR_NO_PARAMETER;

    out = tmp.data;
    return CANLIB_OK;
}
canlib_error canlibSetData(const canlib_address address, const canlib_data data)
{
    const lock_guard dataLock(dataMutex);

    parameter_wrapper& tmp = lookupParameter(getParameterSpecifier(address));
    if (tmp.data.data_len == CANLIB_ERROR_LENGTH)
        return CANLIB_ERROR_NO_PARAMETER;
    if (tmp.data.data_len != data.data_len)
        return CANLIB_ERROR_LENGTH_MISMATCH;

    tmp.data = data;
    return CANLIB_OK;
}

canlib_error canlibAddParameter(canlib_parameter_description description, const canlib_data initialData, const canlib_data dataSafeState)
{
    const lock_guard dataLock(dataMutex);

    uint32_t address = getParameterSpecifier(description.address);

    if (lookupParameter(address).data.data_len != CANLIB_ERROR_LENGTH)
    {
        CANLIB_ERROR("Parameter 0x%04X is already tracked!", address);
        return CANLIB_ERROR_DUPLICATE_PARAMETER;
    }

    if (initialData.data_len == 0)
    {
        CANLIB_ERROR("Invalid data length provided!");
        return CANLIB_ERROR_INVALID_LENGTH;
    }
    if (initialData.data_len == 0 || initialData.data_len > CANLIB_MAX_PACKET_LENGTH)
    {
        CANLIB_ERROR("Data length is invalid!");
        return CANLIB_ERROR_INVALID_LENGTH;
    }
    if (dataSafeState.data_len != 0 && initialData.data_len != dataSafeState.data_len)
    {
        CANLIB_ERROR("Data safe state length is invalid!");
        return CANLIB_ERROR_INVALID_LENGTH;
    }
    if (description.interval == 0)
    {
        if (description.broadcast)
        {
            CANLIB_ERROR("Parameter marked for broadcast, but no interval provided!");
            return CANLIB_ERROR_INVALID_INTERVAL;
        }
        if (description.timeout_handler)
            CANLIB_WARN("Timeout handler provided, but no interval specified!");
        if (description.timeout_recovery_handler)
            CANLIB_WARN("Timeout recovery handler provided, but no interval specified!");
        if (dataSafeState.data_len)
            CANLIB_WARN("Data safe state provided, but no interval specified!");
    }
    if (dataSafeState.data_len && description.broadcast)
        CANLIB_WARN("Data safe state provided, but parameter is marked for boradcast!");

    if (description.broadcast)
    {
        CANLIB_DEBUG("Setting up parameter broadcast (%04X) - Size: %d, interval: %d", address, initialData.data_len, description.interval);
        broadcastParameters.push_back(parameter_wrapper{
            .description = description,
            .data = initialData,
            .data_safe_state = dataSafeState});
    }
    else
    {
        CANLIB_DEBUG("Now tracking parameter (%04X) - Size: %d, timeout: %d", address, initialData.data_len, description.interval);
        standardParameters.push_back(parameter_wrapper{
            .description = description,
            .data = initialData,
            .data_safe_state = dataSafeState});
    }

    return CANLIB_OK;
}
canlib_error canlibRemoveParameter(canlib_address address)
{
    const lock_guard dataLock(dataMutex);

    for (auto it = standardParameters.begin(); it != standardParameters.end(); it++)
    {
        if (address == (*it).description.address)
        {
            standardParameters.erase(it);
            break;
        }
    }
    for (auto it = broadcastParameters.begin(); it != broadcastParameters.end(); it++)
    {
        if (address == (*it).description.address)
        {
            broadcastParameters.erase(it);
            break;
        }
    }

    return CANLIB_OK;
}

canlib_error canlibTransmit(canlib_packet packet, bool useDeviceId)
{
    if (useDeviceId)
    {
        // add the device ID to the address
        packet.address = canlib_address::fromUint32((uint32_t)packet.address | (uint32_t)currentAddress);
    }
    CANLIB_DEBUG("Transmitting message - ID: 0x%08X, length: %d", (uint32_t)packet.address, packet.data.data_len);
    if (!packet.data_request && (packet.data.data_len == 0 || packet.data.data_len > CANLIB_MAX_PACKET_LENGTH))
    {
        CANLIB_ERROR("Data length is invalid!");
        return CANLIB_ERROR_INVALID_LENGTH;
    }
    if (packet.data_request && packet.data.data_len > 0)
        CANLIB_WARN("Data length provided for RTR packet!");

    twai_message_t msg = {};
    msg.extd = true;
    msg.rtr = packet.data_request;
    msg.data_length_code = packet.data.data_len;
    msg.identifier = (uint32_t)packet.address;
#ifdef CANLIB_LOOPBACK
    msg.self = true;
#endif
    memcpy(msg.data, packet.data.data_arr, packet.data.data_len);

    esp_err_t err = twai_transmit(&msg, pdMS_TO_TICKS(CONFIG_CANLIB_BUS_TX_TIME));
    if (err == ESP_OK)
    {
        return CANLIB_OK;  // nothing more to do!
    }

    if (err == ESP_ERR_TIMEOUT)
    {
        CANLIB_ERROR("Queuing message 0x%08X timeout!", msg.identifier);
        return CANLIB_ERROR_TIMEOUT;
    }

    if (err == ESP_ERR_INVALID_STATE)
    {
        if (!txErrorPrinted)
            CANLIB_ERROR("Attempting to transmit, but driver in invalid state!");
        txErrorPrinted = true;
        return CANLIB_ERROR_DRIVER_INVALID_STATE;
    }

    CANLIB_ERROR("Error transmitting: %s", esp_err_to_name(err));
    return CANLIB_ERROR_GENERIC;
}

canlib_error canlibHandle()
{
    if (checkAlerts())
        return CANLIB_OK;

    dataMutex.lock();
    // --- BROADCASTING ---
    TickType_t currentTick = xTaskGetTickCount();
    for (parameter_wrapper& wrapper : broadcastParameters)
    {
        if (currentTick - wrapper.last_packet_tick > pdMS_TO_TICKS(wrapper.description.interval))
        {
            wrapper.last_packet_tick = currentTick;
            canlibTransmit({.address = wrapper.description.address, .data = wrapper.data});
        }
    }

    // --- TIMEOUTS ---
    for (parameter_wrapper& wrapper : standardParameters)
    {
        TickType_t interval = wrapper.description.interval;

        if (interval && !wrapper.timed_out &&
            ((currentTick - wrapper.last_packet_tick) >= pdMS_TO_TICKS(interval * CANLIB_TIMEOUT_COUNT)))
        {
            wrapper.timed_out = true;
            CANLIB_WARN("Parameter 0x%08X timed out!", (uint32_t)wrapper.description.address);
            if (wrapper.data_safe_state.data_len == wrapper.data.data_len)
            {
                memcpy(wrapper.data.data_arr, wrapper.data_safe_state.data_arr, wrapper.data.data_len);
                dataMutex.unlock();
                if (wrapper.description.data_change_handler)
                    wrapper.description.data_change_handler(wrapper.description.address);
                dataMutex.lock();
            }
            dataMutex.unlock();
            if (wrapper.description.timeout_handler)
                wrapper.description.timeout_handler(wrapper.description.address);
            dataMutex.lock();
        }
    }
    dataMutex.unlock();
    // --- RX HANDLING ---
    twai_message_t msg = {};
    esp_err_t err = twai_receive(&msg, 0);

    if (err == ESP_ERR_TIMEOUT)
        return CANLIB_OK;  // nothing to receive
    if (err == ESP_ERR_INVALID_STATE)
    {
        twai_status_info_t status;
        esp_err_t err = twai_get_status_info(&status);
        if (err != ESP_OK)  // driver not installed
        {
            if (!rxErrorPrinted)
                CANLIB_ERROR("Attempting to receive, but driver not installed!");
            rxErrorPrinted = true;
            return CANLIB_ERROR_DRIVER_NOT_INSTALLED;
        }
        if (status.state == TWAI_STATE_STOPPED)
        {
            CANLIB_WARN("Driver not running - re-initialising...");
            canlibStop();
            canlibInit(currentAddress, currentTx, currentRx);
            return CANLIB_OK;
        }

        if (!rxErrorPrinted)
            CANLIB_ERROR("Attempting to receive, but node is in error state!");
        rxErrorPrinted = true;
    }

    // check if something not already handled went wrong
    if (err != ESP_OK)  // no message
    {
        CANLIB_ERROR("Error receiving: %s", esp_err_to_name(err));
        return CANLIB_ERROR_GENERIC;
    }

    dataMutex.lock();
    CANLIB_DEBUG("Message 0x%08X (%02d, %02X) received!", msg.identifier, msg.data_length_code, msg.flags);
    parameter_wrapper& wrapper = lookupParameter(getParameterSpecifier(msg.identifier));
    if (wrapper.data.data_len == CANLIB_ERROR_LENGTH)
    {
        dataMutex.unlock();
        CANLIB_ERROR("No parameter registered to 0x%08X!", getParameterSpecifier(msg.identifier));
        return CANLIB_ERROR_NO_PARAMETER;
    }

    if (msg.rtr)
    {
        dataMutex.unlock();
        CANLIB_DEBUG("RTR - Sending parameter 0x%08X", msg.identifier);
        return canlibTransmit({.address = wrapper.description.address, .data = wrapper.data});
    }

    if (msg.data_length_code != wrapper.data.data_len)
    {
        dataMutex.unlock();
        CANLIB_ERROR("Message 0x%08X length (%d) does not match with stored parameter length (%d)!", msg.identifier, msg.data_length_code, wrapper.data.data_len);
        return CANLIB_ERROR_LENGTH_MISMATCH;
    }

    if (wrapper.description.writable)
        memcpy(wrapper.data.data_arr, msg.data, wrapper.data.data_len);
    else
        CANLIB_WARN("Non-writable parameter 0x%08X received write request!", (uint32_t)wrapper.description.address);

    dataMutex.unlock();

    if (wrapper.description.data_change_handler)
        wrapper.description.data_change_handler(wrapper.description.address);

    wrapper.last_packet_tick = currentTick;
    if (wrapper.timed_out)
    {
        CANLIB_INFO("Parameter 0x%08X timeout reset!", (uint32_t)wrapper.description.address);
        if (wrapper.description.timeout_recovery_handler)
            wrapper.description.timeout_recovery_handler(wrapper.description.address);
    }
    wrapper.timed_out = false;
    return CANLIB_OK;
}