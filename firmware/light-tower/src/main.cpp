#include <Arduino.h>
#include <FastLED.h>

#include <chrono>
#include <hi_can_twai.hpp>
#include <optional>
#include <thread>
#include <vector>

#define DATA_PIN 15

using namespace hi_can;
using namespace addressing;
using namespace addressing::status_light;
using namespace addressing::status_light::control::colour;
using namespace hi_can::parameters::status_light::control::colour;

using namespace std::chrono;
using namespace std::chrono_literals;

constexpr size_t LED_COUNT = 64;
CRGB leds[LED_COUNT];
CRGB last_colour = CRGB::Red;
volatile bool can_healthy = false;

constexpr standard_address_t DEVICE_ADDRESS{
    status_light::SYSTEM_ID,
    status_light::control::SUBSYSTEM_ID,
    status_light::control::colour::DEVICE,
};

std::optional<PacketManager> packet_manager;

void startup_animation();
void handle_light_data(const Packet& packet);
void set_light_colour(const status_light::control::colour::parameter& param, const uint32_t& colour);

void setup()
{
    FastLED.addLeds<WS2812, DATA_PIN, GRB>(leds, LED_COUNT);

    fill_solid(&leds[0], LED_COUNT, CRGB::Black);
    FastLED.show();

    try
    {
        auto& interface = TwaiInterface::get_instance(
            std::make_pair(bsp::CAN_TX_PIN, bsp::CAN_RX_PIN), 0,
            addressing::filter_t{
                .address = static_cast<flagged_address_t>(DEVICE_ADDRESS),  // 0x03800000
                .mask = hi_can::addressing::DEVICE_MASK,
            });
        packet_manager.emplace(interface);
    }
    catch (const std::exception& e)
    {
        printf(std::format("Failed to twai: {}\n", e.what()).c_str());
    }

    packet_manager->set_transmission_config(
        static_cast<flagged_address_t>(
            standard_address_t{DEVICE_ADDRESS,
                               static_cast<uint8_t>(group::RING),
                               static_cast<uint8_t>(parameter::STATUS)}),
        {
            .generator = [=]()
            {
                return rgba_t{static_cast<uint32_t>(0)}.serialize_data();
            },
            .interval = 500ms,
            .should_transmit_immediately = true,
        });
    packet_manager->set_callback(
        filter_t{static_cast<flagged_address_t>(standard_address_t{DEVICE_ADDRESS,
                                                                   static_cast<uint8_t>(group::RING),
                                                                   static_cast<uint8_t>(parameter::RGB)})},
        {
            .data_callback = handle_light_data,
        });

    // startup_animation();
}

void loop()
{
    packet_manager->handle();

    if (!can_healthy)
    {
        // flash red every 500ms
        uint32_t t = millis();
        bool flash_on = (t / 500) % 2 == 0;
        fill_solid(&leds[0], LED_COUNT, flash_on ? CRGB::Red : CRGB::Black);
        FastLED.show();
    }
    
    delay(1);
}

void handle_light_data(const Packet& packet)
{
    try
    {
        // Print received packet info
        standard_address_t address{packet.get_address().address};
        printf("RX: addr=0x%08X sys=%d sub=%d dev=%d grp=%d param=%d data=[",
               (uint32_t)packet.get_address().address,
               address.system,
               address.subsystem,
               address.device,
               address.group,
               address.parameter);
        for (auto b : packet.get_data())
            printf("%02X ", b);
        printf("]\n");

        set_light_colour(
            static_cast<parameter>(standard_address_t(packet.get_address().address).parameter),
            rgba_t{packet.get_data()}.value);
    }
    catch (const std::exception& e)
    {
        printf(std::format("Failed to parse colour packet: {}\n", e.what()).c_str());
    }
}

CRGB uint32_to_crgb(uint32_t raw)
{
    // changing int32_t -> CRGB
    return CRGB((raw >> 16) & 0xFF, (raw >> 8) & 0xFF, raw & 0xFF);
}

void set_light_colour(const status_light::control::colour::parameter& param, const uint32_t& colour)
{
    can_healthy = true;
    switch (param) {
    case parameter::RGB: {                          
        CRGB rgb_colour = uint32_to_crgb(colour);
        fill_solid(&leds[0], LED_COUNT, rgb_colour);
        printf("Changed colour!");
        break;
    }
    default:
        fill_solid(&leds[0], LED_COUNT, CRGB::Red);
        break;
    }
    FastLED.show();
}