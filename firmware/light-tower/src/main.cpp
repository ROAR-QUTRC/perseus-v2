#include <Arduino.h>
#include <FastLED.h>

#include <chrono>
#include <hi_can_twai.hpp>
#include <optional>
#include <thread>
#include <vector>

/* Terminology:
- Band refers to one of the transparent windows on the light tower
- Ring refers to the concentric loop of LEDs inside, usually 2 or more loops of LED strip
*/

#define DATA_PIN 15

constexpr size_t HIGH_DENSITY_LEDS_PER_RING = 30;
constexpr size_t HIGH_DENSITY_RING_COUNT = 3;
constexpr size_t HIGH_DENSITY_LED_COUNT = HIGH_DENSITY_LEDS_PER_RING * HIGH_DENSITY_RING_COUNT;

constexpr size_t LOW_DENSITY_LEDS_PER_RING = 12;
constexpr size_t LOW_DENSITY_RING_COUNT = 2;
constexpr size_t LOW_DENSITY_LED_COUNT = LOW_DENSITY_LEDS_PER_RING * LOW_DENSITY_RING_COUNT;

constexpr size_t LED_COUNT = HIGH_DENSITY_LED_COUNT + (4 * LOW_DENSITY_LED_COUNT);

CRGB leds[LED_COUNT];

using namespace hi_can;
using namespace addressing::legacy;
using namespace addressing::legacy::power::control;
using namespace parameters::legacy::power::control::power_bus;
using namespace std::chrono;
using namespace std::chrono_literals;

const address_t rcb_address{power::SYSTEM_ID,
                            power::control::SUBSYSTEM_ID,
                            static_cast<uint8_t>(device::ROVER_CONTROL_BOARD)};

PowerBusParameterGroup compute_bus{rcb_address, power::control::rcb::groups::COMPUTE_BUS};
PowerBusParameterGroup drive_bus{rcb_address, power::control::rcb::groups::DRIVE_BUS};
PowerBusParameterGroup aux_bus{rcb_address, power::control::rcb::groups::AUX_BUS};
PowerBusParameterGroup spare_bus{rcb_address, power::control::rcb::groups::SPARE_BUS};

std::optional<PacketManager> packet_manager;

void set_ring_status(int main_ring, power_status compute, power_status drive, power_status aux, power_status spare);

void setup()
{
    FastLED.addLeds<WS2812, DATA_PIN, GRB>(leds, LED_COUNT);

    // initialise the CAN interface
    auto& interface = TwaiInterface::get_instance(std::make_pair(bsp::CAN_TX_PIN, bsp::CAN_RX_PIN), 0,
                                                  addressing::filter_t{
                                                      .address = 0x01000000,
                                                      .mask = hi_can::addressing::DEVICE_MASK,
                                                  });
    packet_manager.emplace(interface);
    packet_manager->add_group(compute_bus);
    packet_manager->add_group(drive_bus);
    packet_manager->add_group(aux_bus);
    packet_manager->add_group(spare_bus);
}

void loop()
{
    static steady_clock::time_point last_update = steady_clock::now();

    // TwaiInterface::get_instance().handle();
    packet_manager->handle(false);

    // needs delay in between LED updates so they actually refresh
    if ((steady_clock::now() - last_update) > 50ms)
    {
        last_update = steady_clock::now();
        printf(std::format("Compute: {}  Drive: {}  Aux: {}  Spare: {}\n",
                           static_cast<uint8_t>(compute_bus.get_status().status),
                           static_cast<uint8_t>(drive_bus.get_status().status),
                           static_cast<uint8_t>(aux_bus.get_status().status),
                           static_cast<uint8_t>(spare_bus.get_status().status))
                   .c_str());
        set_ring_status(0, compute_bus.get_status().status, drive_bus.get_status().status, aux_bus.get_status().status, spare_bus.get_status().status);
    }
    // feed WDT
    std::this_thread::sleep_for(1ms);
}

CRGB status_to_color(power_status status)
{
    static bool blink_on = false;
    static steady_clock::time_point last_blink = steady_clock::now();

    if ((steady_clock::now() - last_blink) > 500ms)
    {
        last_blink = steady_clock::now();
        blink_on = !blink_on;
    }

    switch (status)
    {
    case power_status::OFF:
        return CRGB::Black;
    case power_status::ON:
        return CRGB::Green;
    case power_status::PRECHARGING:
        return CRGB::Yellow;
    case power_status::SHORT_CIRCUIT:
        return CRGB::Red;
    case power_status::SWITCH_FAILED:
        return blink_on ? CRGB::OrangeRed : CRGB::Black;
    case power_status::OVERLOAD:
        return CRGB::Blue;
    case power_status::FAULT:
        return blink_on ? CRGB::Red : CRGB::Black;
    default:
        return CRGB::Black;
    }
}
void set_ring_status(int main_ring, power_status compute, power_status drive, power_status aux, power_status spare)
{
    size_t led_index = 0;
    fill_solid(&leds[led_index], HIGH_DENSITY_LED_COUNT, CRGB::Lime);
    led_index += HIGH_DENSITY_LED_COUNT;

    fill_solid(&leds[led_index], LOW_DENSITY_LED_COUNT, status_to_color(compute));
    led_index += LOW_DENSITY_LED_COUNT;
    fill_solid(&leds[led_index], LOW_DENSITY_LED_COUNT, status_to_color(drive));
    led_index += LOW_DENSITY_LED_COUNT;
    fill_solid(&leds[led_index], LOW_DENSITY_LED_COUNT, status_to_color(aux));
    led_index += LOW_DENSITY_LED_COUNT;
    fill_solid(&leds[led_index], LOW_DENSITY_LED_COUNT, status_to_color(spare));

    FastLED.show();
}

void startup_animation()
{
    size_t led_index = 0;
    fill_solid(&leds[led_index], LED_COUNT, CRGB::Black);
    for (int i = 0; i <= LED_COUNT; i++)
    {
        // insert 50ms delay for smoother transition
        leds[led_index] = CRGB::White;
        FastLED.show();
        std::this_thread::sleep_for(5s / LED_COUNT);
    }

    std::vector<size_t> band_sizes = {HIGH_DENSITY_LED_COUNT, LOW_DENSITY_LED_COUNT, LOW_DENSITY_LED_COUNT, LOW_DENSITY_LED_COUNT, LOW_DENSITY_LED_COUNT};
    std::vector<size_t> ring_count = {HIGH_DENSITY_RING_COUNT, LOW_DENSITY_RING_COUNT, LOW_DENSITY_RING_COUNT, LOW_DENSITY_RING_COUNT, LOW_DENSITY_RING_COUNT};
    steady_clock::duration animation_time = 5s;
    int rotation_count = 3;
    steady_clock::duration step_time = animation_time / (rotation_count * HIGH_DENSITY_LED_COUNT);
    float current_pos = 0.0f;

    steady_clock::time_point start_time = steady_clock::now();
    while ((steady_clock::now() - start_time < 10s))
    {
        size_t current_offset = 0;
        for (size_t i = 0; i <= band_sizes.size(); i++)  // Loop over all bands
        {
            size_t band_size = band_sizes[i];
            bool is_reversed = i % 2;
            size_t rotation_offset = band_size - static_cast<size_t>(std::round(current_pos * band_size));

            fill_solid(&leds[led_index], band_size, CRGB::Black);

            for (size_t j = 0; j <= ring_count[i]; j++)
            {
                for (size_t k = 0; k <= (band_size / ring_count[i]) / 2; k++)  // loop over each LED in a half ring (band_size/ring_count = Num of LEDs for a ring)
                {
                    size_t led_offset = (rotation_offset + k) % band_size;
                    leds[led_offset + current_offset + (j * band_size / ring_count[i])] = CRGB::White;
                }
            }
            current_offset += band_size;
        }
        FastLED.show();
        std::this_thread::sleep_for(step_time);
        current_pos += 1.0f / HIGH_DENSITY_LED_COUNT;
    }
}
