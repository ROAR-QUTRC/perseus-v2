#include <Arduino.h>
#include <FastLED.h>

#include <chrono>
#include <hi_can_twai.hpp>
#include <optional>
#include <thread>

#define DATA_PIN 12

constexpr size_t HIGH_DENSITY_LEDS_PER_RING = 29;
constexpr size_t HIGH_DENSITY_RING_COUNT = 3;
constexpr size_t HIGH_DENSITY_LED_COUNT = HIGH_DENSITY_LEDS_PER_RING * HIGH_DENSITY_RING_COUNT;

constexpr size_t LOW_DENSITY_LEDS_PER_RING = 19;
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

const address_t rcbAddress{power::SYSTEM_ID,
                           power::control::SUBSYSTEM_ID,
                           static_cast<uint8_t>(device::ROVER_CONTROL_BOARD)};

PowerBusParameterGroup computeBus{rcbAddress, power::control::rcb::groups::COMPUTE_BUS};
PowerBusParameterGroup driveBus{rcbAddress, power::control::rcb::groups::DRIVE_BUS};
PowerBusParameterGroup auxBux{rcbAddress, power::control::rcb::groups::AUX_BUS};
PowerBusParameterGroup spareBus{rcbAddress, power::control::rcb::groups::SPARE_BUS};

std::optional<PacketManager> packetManager;

void setRingStatus(int mainRing, power_status compute, power_status drive, power_status aux, power_status spare);

void setup()
{
    FastLED.addLeds<WS2812, DATA_PIN, GRB>(leds, LED_COUNT);

    // initialise the CAN interface
    auto& interface = TwaiInterface::getInstance();
    packetManager.emplace(interface);
    packetManager->addGroup(computeBus);
    packetManager->addGroup(driveBus);
    packetManager->addGroup(auxBux);
    packetManager->addGroup(spareBus);
}

void loop()
{
    static steady_clock::time_point lastUpdate = steady_clock::now();

    packetManager->handle(false);

    // needs delay in between LED updates so they actually refresh
    if ((steady_clock::now() - lastUpdate) > 50ms)
    {
        lastUpdate = steady_clock::now();
        setRingStatus(0, computeBus.getStatus().status, driveBus.getStatus().status, auxBux.getStatus().status, spareBus.getStatus().status);
    }
    // feed WDT
    std::this_thread::sleep_for(1ms);
}

CRGB statusToColor(power_status status)
{
    static bool blinkOn = false;
    static steady_clock::time_point lastBlink = steady_clock::now();

    if ((steady_clock::now() - lastBlink) > 500ms)
    {
        lastBlink = steady_clock::now();
        blinkOn = !blinkOn;
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
        return blinkOn ? CRGB::OrangeRed : CRGB::Black;
    case power_status::OVERLOAD:
        return CRGB::Blue;
    case power_status::FAULT:
        return blinkOn ? CRGB::Red : CRGB::Black;
    default:
        return CRGB::Black;
    }
}
void setRingStatus(int mainRing, power_status compute, power_status drive, power_status aux, power_status spare)
{
    size_t ledIndex = 0;
    fill_solid(&leds[ledIndex], HIGH_DENSITY_LED_COUNT, CRGB::Lime);
    ledIndex += HIGH_DENSITY_LED_COUNT;

    fill_solid(&leds[ledIndex], LOW_DENSITY_LED_COUNT, statusToColor(compute));
    ledIndex += LOW_DENSITY_LED_COUNT;
    fill_solid(&leds[ledIndex], LOW_DENSITY_LED_COUNT, statusToColor(drive));
    ledIndex += LOW_DENSITY_LED_COUNT;
    fill_solid(&leds[ledIndex], LOW_DENSITY_LED_COUNT, statusToColor(aux));
    ledIndex += LOW_DENSITY_LED_COUNT;
    fill_solid(&leds[ledIndex], LOW_DENSITY_LED_COUNT, statusToColor(spare));

    FastLED.show();
}