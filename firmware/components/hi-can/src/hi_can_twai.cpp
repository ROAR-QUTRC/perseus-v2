#include "hi_can_twai.hpp"

#include <driver/gpio.h>
#include <unistd.h>

#include <algorithm>
#include <cassert>
#include <cstring>
#include <stdexcept>

#include "sdkconfig.h"

using namespace hi_can;
using namespace std::chrono;
using namespace std::chrono_literals;
using std::string;

TwaiInterface& TwaiInterface::getInstance(pin_pair_t pins, uint8_t controllerId, addressing::filter_t filter)
{
    static std::optional<TwaiInterface> instance;
    if (!instance)
        instance = TwaiInterface(pins, controllerId, filter);
    return *instance;
}

TwaiInterface::TwaiInterface(pin_pair_t pins, uint8_t controllerId, addressing::filter_t filter)
    : _controllerId(controllerId)
{
#ifndef CONFIG_HI_CAN_NO_ACK
    // standard configs at 125kbps
    twai_general_config_t generalConfig = TWAI_GENERAL_CONFIG_DEFAULT_V2(controllerId, std::get<0>(pins), std::get<1>(pins), TWAI_MODE_NORMAL);
#else
    // loopback - not going to get an ACK
    twai_general_config_t generalConfig = TWAI_GENERAL_CONFIG_DEFAULT_V2(controllerId, std::get<0>(pins), std::get<1>(pins), TWAI_MODE_NO_ACK);
#endif

#if defined(CONFIG_HI_CAN_BAUD_1M)
    twai_timing_config_t timingConfig = TWAI_TIMING_CONFIG_1MBITS();
#elif defined(CONFIG_HI_CAN_BAUD_250K)
    twai_timing_config_t timingConfig = TWAI_TIMING_CONFIG_250KBITS();
#elif defined(CONFIG_HI_CAN_BAUD_125K)
    twai_timing_config_t timingConfig = TWAI_TIMING_CONFIG_125KBITS();
#else  // default: 500K
    twai_timing_config_t timingConfig = TWAI_TIMING_CONFIG_500KBITS();
#endif

    twai_filter_config_t filterConfig = {
        .acceptance_code = filter.address.address << 3,
        // TWAI filter requires MSB to be rightmost, and RTR filtering is bit 3
        // Additionally, the acceptance mask is "set bit to ignore" rather than "clear to ignore" like SocketCAN
        .acceptance_mask = ((~filter.mask) << 3) | 0x00000004,
        .single_filter = true,
    };

    if (twai_driver_install_v2(&generalConfig, &timingConfig, &filterConfig, &_twaiBus) == ESP_OK)
    {
        printf("Driver installed\n");
    }
    else
    {
        printf("Failed to install driver\n");
        return;
    }
    if (twai_start_v2(_twaiBus) == ESP_OK)
    {
        printf("Driver started\n");
    }
    else
    {
        printf("Failed to start driver\n");
        return;
    }
    if (twai_reconfigure_alerts_v2(_twaiBus, TWAI_ALERT_ALL, nullptr) == ESP_OK)
    {
        printf("Alerts reconfigured\n");
    }
    else
    {
        printf("Failed to reconfigure alerts\n");
        return;
    }
}

TwaiInterface::~TwaiInterface()
{
    if (_controllerId == INVALID_INTERFACE_ID)
        return;
    twai_stop_v2(_twaiBus);
    twai_driver_uninstall_v2(_twaiBus);
}
void TwaiInterface::transmit(const Packet& packet)
{
    const auto address = packet.getAddress();
    twai_message_t message{
        .identifier = address.address,
        .data_length_code = static_cast<uint8_t>(packet.getDataLen()),
    };
    std::copy_n(packet.getData().cbegin(), packet.getDataLen(), message.data);
    message.extd = address.isExtended;
    message.rtr = address.isRtr;
    message.ss = false;            // not single-shot (re-try on error)
    message.self = false;          // not self-reception
    message.dlc_non_comp = false;  // data length code is <= 8

    if (esp_err_t err = twai_transmit_v2(_twaiBus, &message, pdMS_TO_TICKS(CONFIG_HI_CAN_BUS_TX_TIME));
        err != ESP_OK)
    {
        throw std::runtime_error(std::format("Failed to transmit packet {:#08x}: {}",
                                             packet.getAddress().address, esp_err_to_name(err)));
    }
}
std::optional<Packet> TwaiInterface::receive(bool blocking)
{
    twai_message_t message;
    if (esp_err_t err = twai_receive_v2(_twaiBus, &message, blocking ? portMAX_DELAY : 0); err != ESP_OK)
    {
        if (err == ESP_ERR_TIMEOUT)
            return std::nullopt;
        if (err == ESP_ERR_INVALID_ARG)
            throw std::runtime_error("Invalid arguments for TWAI receive");
        if (err == ESP_ERR_INVALID_STATE)
            throw std::runtime_error("TWAI driver not installed");
    }
    Packet packet{
        addressing::flagged_address_t(message.identifier, message.rtr, false, message.extd),
        message.data, message.data_length_code};

    if (_receiveCallback)
        _receiveCallback(packet);

    return packet;
}

void TwaiInterface::handle()
{
    uint32_t alerts;
    if (twai_read_alerts_v2(_twaiBus, &alerts, 0) != ESP_OK)
    {
        throw std::runtime_error("Failed to read alerts");
    }
    if (alerts & TWAI_ALERT_BUS_OFF)
    {
        if (_recoveryAttemptCount++ >= CONFIG_HI_CAN_BUS_RECOVERY_ATTEMPTS)
        {
            throw std::runtime_error("Recovery attempts failed");
        }
        if (twai_initiate_recovery_v2(_twaiBus) != ESP_OK)
        {
            throw std::runtime_error("Failed to initiate error recovery");
        }
    }
    else if (alerts & TWAI_ALERT_BUS_RECOVERED)
        _recoveryAttemptCount = 0;
}

TwaiInterface& hi_can::TwaiInterface::addFilter(const addressing::filter_t& address)
{
    FilteredCanInterface::addFilter(address);
    // TODO: IMPLEMENT
    return *this;
}

TwaiInterface& hi_can::TwaiInterface::removeFilter(const addressing::filter_t& address)
{
    FilteredCanInterface::removeFilter(address);
    // TODO: IMPLEMENT
    return *this;
}
