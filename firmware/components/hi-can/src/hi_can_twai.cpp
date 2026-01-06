#include "hi_can_twai.hpp"

#include <driver/gpio.h>
#include <unistd.h>

#include <algorithm>
#include <cassert>
#include <cstring>
#include <format>
#include <stdexcept>

#include "sdkconfig.h"

using namespace bsp;
using namespace hi_can;
using namespace std::chrono;
using namespace std::chrono_literals;
using std::string;

TwaiInterface& TwaiInterface::get_instance(pin_pair_t pins, uint8_t controller_id, addressing::filter_t filter)
{
    static std::optional<TwaiInterface> instance;
    if (!instance)
        instance = TwaiInterface(pins, controller_id, filter);
    return *instance;
}

TwaiInterface::TwaiInterface(pin_pair_t pins, uint8_t controller_id, addressing::filter_t filter)
    : _controller_id(controller_id)
{
#ifndef CONFIG_HI_CAN_NO_ACK
    // standard configs at 125kbps
    twai_general_config_t general_config = TWAI_GENERAL_CONFIG_DEFAULT_V2(controller_id, std::get<0>(pins), std::get<1>(pins), TWAI_MODE_NORMAL);
#else
    // loopback - not going to get an ACK
    twai_general_config_t general_config = TWAI_GENERAL_CONFIG_DEFAULT_V2(controller_id, std::get<0>(pins), std::get<1>(pins), TWAI_MODE_NO_ACK);
#endif

#if defined(CONFIG_HI_CAN_BAUD_1M)
    twai_timing_config_t timing_config = TWAI_TIMING_CONFIG_1MBITS();
#elif defined(CONFIG_HI_CAN_BAUD_250K)
    twai_timing_config_t timing_config = TWAI_TIMING_CONFIG_250KBITS();
#elif defined(CONFIG_HI_CAN_BAUD_125K)
    twai_timing_config_t timing_config = TWAI_TIMING_CONFIG_125KBITS();
#else  // default: 500K
    twai_timing_config_t timing_config = TWAI_TIMING_CONFIG_500KBITS();
#endif

    twai_filter_config_t filter_config = {
        .acceptance_code = filter.address.address << 3,
        // TWAI filter requires MSB to be rightmost, and RTR filtering is bit 3
        // Additionally, the acceptance mask is "set bit to ignore" rather than "clear to ignore" like SocketCAN
        .acceptance_mask = ((~filter.mask) << 3) | 0x00000004,
        .single_filter = true,
    };

    if (twai_driver_install_v2(&general_config, &timing_config, &filter_config, &_twai_bus) == ESP_OK)
    {
        printf("Driver installed\n");
    }
    else
    {
        printf("Failed to install driver\n");
        return;
    }
    if (twai_start_v2(_twai_bus) == ESP_OK)
    {
        printf("Driver started\n");
    }
    else
    {
        printf("Failed to start driver\n");
        return;
    }
    if (twai_reconfigure_alerts_v2(_twai_bus, TWAI_ALERT_ALL, nullptr) == ESP_OK)
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
    if (_controller_id == INVALID_INTERFACE_ID)
        return;
    twai_stop_v2(_twai_bus);
    twai_driver_uninstall_v2(_twai_bus);
}
void TwaiInterface::transmit(const Packet& packet)
{
    const auto address = packet.get_address();
    twai_message_t message{
        .identifier = address.address,
        .data_length_code = static_cast<uint8_t>(packet.get_data_len()),
    };
    std::copy_n(packet.get_data().cbegin(), packet.get_data_len(), message.data);
    message.extd = address.is_extended;
    message.rtr = address.is_rtr;
    message.ss = false;            // not single-shot (re-try on error)
    message.self = false;          // not self-reception
    message.dlc_non_comp = false;  // data length code is <= 8

    if (esp_err_t err = twai_transmit_v2(_twai_bus, &message, pdMS_TO_TICKS(CONFIG_HI_CAN_BUS_TX_TIME));
        err != ESP_OK)
    {
        throw std::runtime_error(std::format("Failed to transmit packet {:#08x}: {}",
                                             packet.get_address().address, esp_err_to_name(err)));
    }
}
std::optional<Packet> TwaiInterface::receive(bool blocking)
{
    twai_message_t message;
    if (esp_err_t err = twai_receive_v2(_twai_bus, &message, blocking ? portMAX_DELAY : 0); err != ESP_OK)
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

    if (_receive_callback)
        _receive_callback(packet);

    return packet;
}

void TwaiInterface::handle()
{
    uint32_t alerts;
    if (twai_read_alerts_v2(_twai_bus, &alerts, 0) != ESP_OK)
    {
        throw std::runtime_error("Failed to read alerts");
    }
    if (alerts & TWAI_ALERT_BUS_OFF)
    {
        if (_recovery_attempt_count++ >= CONFIG_HI_CAN_BUS_RECOVERY_ATTEMPTS)
        {
            throw std::runtime_error("Recovery attempts failed");
        }
        if (twai_initiate_recovery_v2(_twai_bus) != ESP_OK)
        {
            throw std::runtime_error("Failed to initiate error recovery");
        }
    }
    else if (alerts & TWAI_ALERT_BUS_RECOVERED)
        _recovery_attempt_count = 0;
}

TwaiInterface& hi_can::TwaiInterface::add_filter(const addressing::filter_t& address)
{
    FilteredCanInterface::add_filter(address);
    // TODO: IMPLEMENT
    return *this;
}

TwaiInterface& hi_can::TwaiInterface::remove_filter(const addressing::filter_t& address)
{
    FilteredCanInterface::remove_filter(address);
    // TODO: IMPLEMENT
    return *this;
}
