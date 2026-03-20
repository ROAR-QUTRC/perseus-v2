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
            .interval = 100ms,
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
    delay(1);
}

void handle_light_data(const Packet& packet)
{
    printf("Hi");
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
    switch (param)
    {
    case parameter::RGB:
    {
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

void startup_animation()
{
    // 1. Wipe red around the ring
    for (int i = 0; i < LED_COUNT; i++)
    {
        leds[i] = CRGB::Red;
        FastLED.show();
        delay(18);
    }

    // 2. Wipe green over the top
    for (int i = 0; i < LED_COUNT; i++)
    {
        leds[i] = CRGB::Green;
        FastLED.show();
        delay(18);
    }

    // 3. Wipe blue over the top
    for (int i = 0; i < LED_COUNT; i++)
    {
        leds[i] = CRGB::Blue;
        FastLED.show();
        delay(18);
    }

    // 4. Rainbow spin — two full cycles
    for (int cycle = 0; cycle < 512; cycle++)
    {
        for (int i = 0; i < LED_COUNT; i++)
        {
            leds[i] = CHSV((cycle * 3 + i * (256 / LED_COUNT)) & 0xFF, 255, 255);
        }
        FastLED.show();
        delay(8);
    }

    // 5. Breathe white in and out twice
    for (int repeat = 0; repeat < 2; repeat++)
    {
        for (int v = 0; v < 255; v++)
        {
            fill_solid(&leds[0], LED_COUNT, CHSV(0, 0, v));
            FastLED.show();
            delay(4);
        }
        for (int v = 255; v >= 0; v--)
        {
            fill_solid(&leds[0], LED_COUNT, CHSV(0, 0, v));
            FastLED.show();
            delay(4);
        }
    }

    // 6. Fade up to last_colour (ready state)
    for (int v = 0; v < 255; v++)
    {
        CRGB scaled = last_colour;
        nscale8(&scaled, 1, v);
        fill_solid(&leds[0], LED_COUNT, scaled);
        FastLED.show();
        delay(4);
    }
    fill_solid(&leds[0], LED_COUNT, last_colour);
    FastLED.show();
}