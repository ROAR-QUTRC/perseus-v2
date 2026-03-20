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

constexpr size_t LED_COUNT = 60;
CRGB leds[LED_COUNT];
CRGB last_colour = CRGB::Red;

const standard_address_t light_address{
    status_light::SYSTEM_ID,
    status_light::control::SUBSYSTEM_ID,
    status_light::control::colour::DEVICE,
};

std::optional<PacketManager> packet_manager;

void startup_animation();
void handle_light_data(const Packet& packet);
void set_light_colour(const status_light::control::colour::group& group, const rgba_t& colour);

void setup()
{
    FastLED.addLeds<WS2812, DATA_PIN, GRB>(leds, LED_COUNT);

    auto& interface = TwaiInterface::get_instance(std::make_pair(bsp::CAN_TX_PIN, bsp::CAN_RX_PIN), 0,
                                                  addressing::filter_t{
                                                      .address = 0x01000000,
                                                      .mask = hi_can::addressing::DEVICE_MASK,
                                                  });
    packet_manager.emplace(interface);
    packet_manager->set_callback(
        filter_t{static_cast<flagged_address_t>(
            standard_address_t{light_address,
                               static_cast<uint8_t>(group::RING),
                               static_cast<uint8_t>(parameter::RGB)})},
        {
            .data_callback = handle_light_data,
        });                                            

    startup_animation();
}

void loop()
{
    packet_manager->handle();
    delay(1);
}

void handle_light_data(const Packet& packet) 
{
    try
    {
        standard_address_t address{packet.get_address().address};
        set_light_colour(
            static_cast<group>(standard_address_t(packet.get_address().address).parameter),
            rgba_t{packet.get_data()});
    }
    catch (const std::exception& e)
    {
        printf(std::format("Failed to parse colour packet: {}\n", e.what()).c_str());
    }
}

CRGB rgba_to_crgb(rgba_t colour)
{
    // changing rgba_t -> CRGB
    int32_t raw = colour.value;
    return CRGB((raw >> 16) & 0xFF, (raw >> 8) & 0xFF, raw & 0xFF);
}

void set_light_colour(const status_light::control::colour::group& group, const rgba_t& colour)
{
    switch (group) {
    case group::RING: {                          
        CRGB rgb_colour = rgba_to_crgb(colour);
        fill_solid(&leds[0], LED_COUNT, rgb_colour);
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
    for (int i = 0; i < LED_COUNT; i++) {
        leds[i] = CRGB::Red;
        FastLED.show();
        delay(18);
    }

    // 2. Wipe green over the top
    for (int i = 0; i < LED_COUNT; i++) {
        leds[i] = CRGB::Green;
        FastLED.show();
        delay(18);
    }

    // 3. Wipe blue over the top
    for (int i = 0; i < LED_COUNT; i++) {
        leds[i] = CRGB::Blue;
        FastLED.show();
        delay(18);
    }

    // 4. Rainbow spin — two full cycles
    for (int cycle = 0; cycle < 512; cycle++) {
        for (int i = 0; i < LED_COUNT; i++) {
            leds[i] = CHSV((cycle * 3 + i * (256 / LED_COUNT)) & 0xFF, 255, 255);
        }
        FastLED.show();
        delay(8);
    }

    // 5. Breathe white in and out twice
    for (int repeat = 0; repeat < 2; repeat++) {
        for (int v = 0; v < 255; v++) {
            fill_solid(&leds[0], LED_COUNT, CHSV(0, 0, v));
            FastLED.show();
            delay(4);
        }
        for (int v = 255; v >= 0; v--) {
            fill_solid(&leds[0], LED_COUNT, CHSV(0, 0, v));
            FastLED.show();
            delay(4);
        }
    }

    // 6. Fade up to last_colour (ready state)
    for (int v = 0; v < 255; v++) {
        CRGB scaled = last_colour;
        nscale8(&scaled, 1, v);
        fill_solid(&leds[0], LED_COUNT, scaled);
        FastLED.show();
        delay(4);
    }
    fill_solid(&leds[0], LED_COUNT, last_colour);
    FastLED.show();
}