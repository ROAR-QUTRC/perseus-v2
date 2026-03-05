#include <chrono>
#include <optional>
#include <string>
#include <utility>

// #include "Arduino.h"
#include "FastLED.h"
#include "hi_can_address.hpp"
#include "hi_can_packet.hpp"
#include "hi_can_twai.hpp"
using namespace hi_can;
using namespace hi_can::addressing;

std::optional<PacketManager> packet_manager;

const int NUMBER_OF_LEDS = 24;
const int DATA_PIN = 15;
const std::chrono::duration timeout = std::chrono::milliseconds(1000);
CRGB leds[NUMBER_OF_LEDS];

shared::status::light_ring::group light_status;

void fill(CRGB color)
{
    for (int i = 0; i < NUMBER_OF_LEDS; i++)
    {
        leds[i] = color;
    }
}

extern "C" int app_main()
{
    FastLED.addLeds<WS2812, DATA_PIN, GRB>(leds, NUMBER_OF_LEDS);
    try
    {
        auto& can_interface = TwaiInterface::get_instance(std::make_pair(bsp::CAN_TX_PIN, bsp::CAN_RX_PIN), 0, addressing::filter_t{});
        packet_manager.emplace(can_interface);

        packet_manager->set_callback(filter_t{.address = static_cast<flagged_address_t>(standard_address_t(shared::SYSTEM_ID, shared::status::SUBSYSTEM_ID, shared::status::light_ring::DEVICE_ID)), .mask = DEVICE_MASK}, PacketManager::callback_config_t{
                                                                                                                                                                                                                               .data_callback = [](const Packet& packet)
                                                                                                                                                                                                                               {
                raw_address_t packet_address = static_cast<raw_address_t>(packet.get_address());
                shared::status::light_ring::group received_color = static_cast<shared::status::light_ring::group>((packet_address & (GROUP_MASK & !DEVICE_MASK)) >> GROUP_ADDRESS_POS);
                shared::status::light_ring::parameter received_command = static_cast<shared::status::light_ring::parameter>(packet_address & !GROUP_MASK);
                light_status = static_cast<shared::status::light_ring::group>(static_cast<bool>(received_command) ? (static_cast<uint8_t>(light_status) | static_cast<uint8_t>(received_color)) : (static_cast<uint8_t>(light_status) & !static_cast<uint8_t>(received_color))); },
                                                                                                                                                                                                                               .timeout_callback = []()
                                                                                                                                                                                                                               { light_status = shared::status::light_ring::group::RED; },
                                                                                                                                                                                                                               .timeout_recovery_callback = [](const Packet& packet)
                                                                                                                                                                                                                               { light_status = static_cast<shared::status::light_ring::group>(static_cast<uint8_t>(light_status) & !static_cast<uint8_t>(shared::status::light_ring::group::RED));  raw_address_t packet_address = static_cast<raw_address_t>(packet.get_address());
                shared::status::light_ring::group received_color = static_cast<shared::status::light_ring::group>((packet_address & (GROUP_MASK & !DEVICE_MASK)) >> GROUP_ADDRESS_POS);
                shared::status::light_ring::parameter received_command = static_cast<shared::status::light_ring::parameter>(packet_address & !GROUP_MASK);
                light_status = static_cast<shared::status::light_ring::group>(static_cast<bool>(received_command) ? (static_cast<uint8_t>(light_status) | static_cast<uint8_t>(received_color)) : (static_cast<uint8_t>(light_status) & !static_cast<uint8_t>(received_color))); },
                                                                                                                                                                                                                               .timeout = timeout,
                                                                                                                                                                                                                           });
    }
    catch (const std::exception& e)
    {
        printf("Failed to initialise CAN bus: %s", e.what());
        return 1;
    }
    while (1)
    {
        packet_manager->handle();
        if ((static_cast<uint8_t>(light_status) % 2) || (static_cast<uint8_t>(light_status) & static_cast<uint8_t>(shared::status::light_ring::group::RED)))
        {
            fill(CRGB::Red);
        }
        else
        {
            switch (light_status)
            {
            case shared::status::light_ring::group::WHITE:
                fill(CRGB::White);
                break;
            case shared::status::light_ring::group::BLUE:
                fill(CRGB::Blue);
                break;
            case shared::status::light_ring::group::CYAN:
                fill(CRGB::Cyan);
                break;
            case shared::status::light_ring::group::GREEN:
                fill(CRGB::Green);
                break;
            case shared::status::light_ring::group::YELLOW:
                fill(CRGB::Yellow);
                break;
            default:
                fill(CRGB::Red);
            }
        }
    }
}
