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

// Note: In case of changes, startupAnimation will likely need to be changed to reflect modifications to LED_COUNT
// Note: LED setup for new light tower assumed to be 2 low density rings as 1 band
constexpr size_t LED_COUNT = LOW_DENSITY_LED_COUNT;  // = HIGH_DENSITY_LED_COUNT + (4 * LOW_DENSITY_LED_COUNT);

constexpr uint32_t COLOUR_AMBER_HEX = 0xFFBF00;

constexpr uint8_t OffsetTipPoint_OddCheck = (LOW_DENSITY_LEDS_PER_RING % 2 != 0);
constexpr uint8_t OffsetTipPoint = (LOW_DENSITY_LEDS_PER_RING >> 1) + OffsetTipPoint_OddCheck;

constexpr uint8_t QUT_MORSE_CODE_bm = 0b1101 001 1;  // QUT --.- ..- -
constexpr uint8_t QUT_MORSE_CODE_CHAR_END_bm = 0b0001 001 1;

CRGB leds[LED_COUNT];

using namespace hi_can;
using namespace addressing::legacy;
using namespace addressing::legacy::power::control;
using namespace parameters::legacy::power::control::power_bus;
using namespace std::chrono;
using namespace std::chrono_literals;

// const address_t rcbAddress{power::SYSTEM_ID,
//                            power::control::SUBSYSTEM_ID,
//                            static_cast<uint8_t>(device::ROVER_CONTROL_BOARD)};

// PowerBusParameterGroup computeBus{rcbAddress, power::control::rcb::groups::COMPUTE_BUS};
// PowerBusParameterGroup driveBus{rcbAddress, power::control::rcb::groups::DRIVE_BUS};
// PowerBusParameterGroup auxBus{rcbAddress, power::control::rcb::groups::AUX_BUS};
// PowerBusParameterGroup spareBus{rcbAddress, power::control::rcb::groups::SPARE_BUS};

// std::optional<PacketManager> packetManager;

// void setRingStatus(int mainRing, power_status compute, power_status drive, power_status aux, power_status spare);

void setup()
{
    FastLED.addLeds<WS2812, DATA_PIN, GRB>(leds, LED_COUNT);

    startupAnimation();

    /*
    // initialise the CAN interface
    auto& interface = TwaiInterface::getInstance(std::make_pair(bsp::CAN_TX_PIN, bsp::CAN_RX_PIN), 0,
                                                 addressing::filter_t{
                                                     .address = 0x01000000,
                                                     .mask = hi_can::addressing::DEVICE_MASK,
                                                 });
    packetManager.emplace(interface);
    packetManager->addGroup(computeBus);
    packetManager->addGroup(driveBus);
    packetManager->addGroup(auxBus);
    packetManager->addGroup(spareBus);
    */
}

void loop()
{
    static steady_clock::time_point lastUpdate = steady_clock::now();

    // TwaiInterface::getInstance().handle();
    // packetManager->handle(false);

    // needs delay in between LED updates so they actually refresh
    if ((steady_clock::now() - lastUpdate) > 50ms)
    {
        lastUpdate = steady_clock::now();

        fill_solid(&leds[0], LOW_DENSITY_LED_COUNT, CRBG(COLOUR_AMBER_HEX));

        FastLED.show();

        /*
        printf(std::format("Compute: {}  Drive: {}  Aux: {}  Spare: {}\n",
                           static_cast<uint8_t>(computeBus.getStatus().status),
                           static_cast<uint8_t>(driveBus.getStatus().status),
                           static_cast<uint8_t>(auxBus.getStatus().status),
                           static_cast<uint8_t>(spareBus.getStatus().status))
                   .c_str());
        setRingStatus(0, computeBus.getStatus().status, driveBus.getStatus().status, auxBus.getStatus().status, spareBus.getStatus().status);
        */
    }
    // feed WDT
    std::this_thread::sleep_for(1ms);
}

// Note: This will only work if the number of LEDs per ring is 12 and there's only 2 rings
// Displays the morse code for QUT with the light tower (Shorts on the first LED row and Longs on the second LED row)
void startupAnimation()
{
    if ((LED_COUNT >> 1) == 12)  // Just a double check so it doesnt look silly if the LED configuration is changed
    {
        fill_solid(&leds[0], LED_COUNT, CRGB::Black);

        steady_clock::duration animationTime = 6s;
        int cycles = 5;
        steady_clock::duration stepTime = animationTime / (cycles * LOW_DENSITY_LEDS_PER_RING);

        size_t ledIndex = 0;
        uint8_t ledOffset = 0;  // Offset MAX is LOW_DENSITY_RING_COUNT - 1, at Offset MAX set offset to 0
        steady_clock::time_point startTime = steady_clock::now();
        while ((steady_clock::now() - startTime < 12s))
        {
            for (int i = 0; i < 8; i++)
            {
                ledIndex = i - ledOffset - OffsetTipPoint_OddCheck + (i < OffsetTipPoint ? LOW_DENSITY_LEDS_PER_RING : 0);

                if (QUT_MORSE_CODE_bm & (0b1 >> i))
                {
                    leds[ledIndex] = CRGB::Black;
                    leds[ledIndex + LOW_DENSITY_LEDS_PER_RING] = CRGB::White;
                }
                else
                {
                    leds[ledIndex] = CRGB::White;
                    leds[ledIndex + LOW_DENSITY_LEDS_PER_RING] = CRGB::Black;
                }

                if (QUT_MORSE_CODE_CHAR_END_bm & (0b1 >> i))
                {
                    leds[ledIndex + 1] = CRGB::Black;
                    leds[ledIndex + 1 + LOW_DENSITY_LEDS_PER_RING] = CRGB::Black;

                    if (i == 7)
                    {
                        leds[ledIndex + 2] = CRGB::Black;
                        leds[ledIndex + 2 + LOW_DENSITY_LEDS_PER_RING] = CRGB::Black;
                    }
                }
            }

            ledOffset = (ledOffset + 2 < LOW_DENSITY_LEDS_PER_RING - 1) ? ledOffset + 1 : 0;

            std::this_thread::sleep_for(stepTime);
        }
        FastLED.show();
    }
}

/*
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

void startupAnimation()
{
    size_t ledIndex = 0;
    fill_solid(&leds[ledIndex], LED_COUNT, CRGB::Black);
    for (int i = 0; i <= LED_COUNT; i++)
    {
        // insert 50ms delay for smoother transition
        leds[ledIndex] = CRGB::White;
        FastLED.show();
        std::this_thread::sleep_for(5s / LED_COUNT);
    }

    std::vector<size_t> bandSizes = {HIGH_DENSITY_LED_COUNT, LOW_DENSITY_LED_COUNT, LOW_DENSITY_LED_COUNT, LOW_DENSITY_LED_COUNT, LOW_DENSITY_LED_COUNT};
    std::vector<size_t> ringCount = {HIGH_DENSITY_RING_COUNT, LOW_DENSITY_RING_COUNT, LOW_DENSITY_RING_COUNT, LOW_DENSITY_RING_COUNT, LOW_DENSITY_RING_COUNT};
    steady_clock::duration animationTime = 5s;
    int rotationCount = 3;
    steady_clock::duration stepTime = animationTime / (rotationCount * HIGH_DENSITY_LED_COUNT);
    float currentPos = 0.0f;

    steady_clock::time_point startTime = steady_clock::now();
    while ((steady_clock::now() - startTime < 10s))
    {
        size_t currentOffset = 0;
        for (size_t i = 0; i <= bandSizes.size(); i++)  // Loop over all bands
        {
            size_t bandSize = bandSizes[i];
            bool isReversed = i % 2;
            size_t rotationOffset = bandSize - static_cast<size_t>(std::round(currentPos * bandSize));

            fill_solid(&leds[ledIndex], bandSize, CRGB::Black);

            for (size_t j = 0; j <= ringCount[i]; j++)
            {
                for (size_t k = 0; k <= (bandSize / ringCount[i]) / 2; k++)  // loop over each LED in a half ring (bandSize/ringCount = Num of LEDs for a ring)
                {
                    size_t ledOffset = (rotationOffset + k) % bandSize;
                    leds[ledOffset + currentOffset + (j * bandSize / ringCount[i])] = CRGB::White;
                }
            }
            currentOffset += bandSize;
        }
        FastLED.show();
        std::this_thread::sleep_for(stepTime);
        currentPos += 1.0f / HIGH_DENSITY_LED_COUNT;
    }
}
*/