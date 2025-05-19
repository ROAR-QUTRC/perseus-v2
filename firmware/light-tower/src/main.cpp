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

#define DATA_PIN GPIO_NUM_47

constexpr size_t HIGH_DENSITY_LEDS_PER_RING = 30;
constexpr size_t HIGH_DENSITY_RING_COUNT = 3;
constexpr size_t HIGH_DENSITY_LED_COUNT = HIGH_DENSITY_LEDS_PER_RING * HIGH_DENSITY_RING_COUNT;

constexpr size_t LOW_DENSITY_LEDS_PER_RING = 12;
constexpr size_t LOW_DENSITY_RING_COUNT = 2;
constexpr size_t LOW_DENSITY_LED_COUNT = LOW_DENSITY_LEDS_PER_RING * LOW_DENSITY_RING_COUNT;

constexpr size_t LED_COUNT = HIGH_DENSITY_LED_COUNT + (4 * LOW_DENSITY_LED_COUNT);

CRGB leds[LED_COUNT];
size_t ledRotationOffset = 0;

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
PowerBusParameterGroup auxBus{rcbAddress, power::control::rcb::groups::AUX_BUS};
PowerBusParameterGroup spareBus{rcbAddress, power::control::rcb::groups::SPARE_BUS};

std::optional<PacketManager> packetManager;

void setRingStatus(int mainRing, power_status compute, power_status drive, power_status aux, power_status spare);
void startupAnimation();

std::mutex ledMutex;
void ledUpdateTask(void* args)
{
    steady_clock::time_point lastUpdate = steady_clock::now();

    while (true)
    {
        // needs delay in between LED updates so they actually refresh
        if ((steady_clock::now() - lastUpdate) > 250ms)
        {
            ledRotationOffset++;
            ledRotationOffset %= HIGH_DENSITY_LEDS_PER_RING;
            std::lock_guard<std::mutex> lock(ledMutex);
            lastUpdate = steady_clock::now();
            printf(std::format("Compute: {}  Drive: {}  Aux: {}  Spare: {}\n",
                               static_cast<uint8_t>(computeBus.getStatus().status),
                               static_cast<uint8_t>(driveBus.getStatus().status),
                               static_cast<uint8_t>(auxBus.getStatus().status),
                               static_cast<uint8_t>(spareBus.getStatus().status))
                       .c_str());
            setRingStatus(0, computeBus.getStatus().status, driveBus.getStatus().status, auxBus.getStatus().status, spareBus.getStatus().status);
        }

        // feed WDT
        std::this_thread::sleep_for(1ms);
    }
    vTaskDelete(nullptr);
}

void setup()
{
    FastLED.addLeds<WS2812, DATA_PIN, GRB>(leds, LED_COUNT);
    // initialise the CAN interface
    // tx 35 rx 36
    auto& interface = TwaiInterface::getInstance(std::make_pair(GPIO_NUM_35, GPIO_NUM_36), 0,
                                                 addressing::filter_t{
                                                     .address = 0x01000000,
                                                     .mask = hi_can::addressing::DEVICE_MASK,
                                                 });
    packetManager.emplace(interface);
    packetManager->addGroup(computeBus);
    packetManager->addGroup(driveBus);
    packetManager->addGroup(auxBus);
    packetManager->addGroup(spareBus);

    startupAnimation();
    xTaskCreatePinnedToCore(ledUpdateTask, "LED Update Task", 4096, nullptr, 1, nullptr, 0);
}

void loop()
{
    static steady_clock::time_point lastUpdate = steady_clock::now();
    static steady_clock::time_point lastSend = steady_clock::now();

    auto& instance = TwaiInterface::getInstance(std::make_pair(GPIO_NUM_35, GPIO_NUM_36), 0);
    try
    {
        instance.handle();
    }
    catch (std::exception& e)
    {
        printf("Failed to handle TWAI interface: %s\n", e.what());
    }

    try
    {
        std::lock_guard<std::mutex> lock(ledMutex);
        packetManager->handle(false);
    }
    catch (std::exception& e)
    {
        printf("Failed to handle packet manager: %s\n", e.what());
    }

    if ((steady_clock::now() - lastSend) > 10ms)
    {
        lastSend = steady_clock::now();
        try
        {
            twai_status_info_t status;
            twai_get_status_info(&status);
            // printf("TWAI state 0x%X\n", status.state);

            instance.transmit(
                Packet{
                    static_cast<addressing::flagged_address_t>(rcbAddress),
                    status_t{}.serializeData(),
                });
        }
        catch (std::exception& e)
        {
            printf("Failed to send packet: %s\n", e.what());
        }
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

    for (size_t i = 0; i < HIGH_DENSITY_RING_COUNT; i++)
    {
        size_t ledIndex = ledRotationOffset + (i * HIGH_DENSITY_LEDS_PER_RING);
        leds[ledIndex] = CRGB::White;
    }

    FastLED.show();
}

void setRing(size_t ringIndex, const CRGB& color)
{
    size_t ledOffset = 0;
    for (size_t i = 0; i < ringIndex; i++)
    {
        if (i < 3)
            ledOffset += HIGH_DENSITY_LEDS_PER_RING;
        else
            ledOffset += LOW_DENSITY_LEDS_PER_RING;
    }
    size_t ringSize = (ringIndex < 3) ? HIGH_DENSITY_LEDS_PER_RING : LOW_DENSITY_LEDS_PER_RING;
    fill_solid(&leds[ledOffset], ringSize, color);
}
void startupAnimation()
{
    static constexpr steady_clock::duration animationTime = 5s;
    fill_solid(leds, LED_COUNT, CRGB::Black);
    for (size_t i = 0; i < LED_COUNT; i++)
    {
        fill_solid(leds, i, CHSV(i * 255 / (LED_COUNT - 1), 255, 255));
        FastLED.show();
        std::this_thread::sleep_for(animationTime / LED_COUNT);
    }

    std::vector<size_t> bandSizes = {HIGH_DENSITY_LEDS_PER_RING,
                                     LOW_DENSITY_LEDS_PER_RING, LOW_DENSITY_LEDS_PER_RING,
                                     LOW_DENSITY_LEDS_PER_RING, LOW_DENSITY_LEDS_PER_RING};
    std::vector<size_t> ringCounts = {HIGH_DENSITY_RING_COUNT,
                                      LOW_DENSITY_RING_COUNT, LOW_DENSITY_RING_COUNT,
                                      LOW_DENSITY_RING_COUNT, LOW_DENSITY_RING_COUNT};

    size_t totalRingCount = HIGH_DENSITY_RING_COUNT + (4 * LOW_DENSITY_RING_COUNT);
    size_t ringDropStepCount = (totalRingCount * (totalRingCount + 1) / 2) + totalRingCount;
    steady_clock::duration ringDropStepTime = animationTime / ringDropStepCount;
    steady_clock::duration ringDropIntermediateDelay = 200ms;
    for (size_t filledRings = 0; filledRings < totalRingCount; filledRings++)
    {
        for (size_t droppingRing = 0; droppingRing < (totalRingCount - filledRings); droppingRing++)
        {
            fill_solid(leds, LED_COUNT, CRGB::Black);
            for (size_t i = 0; i < filledRings; i++)
                setRing(totalRingCount - i - 1, CRGB::Cyan);
            setRing(droppingRing, CRGB::OrangeRed);
            FastLED.show();
            std::this_thread::sleep_for(ringDropStepTime);
        }
        for (size_t i = 0; i < filledRings; i++)
            setRing(totalRingCount - i - 1, CRGB::Cyan);
        std::this_thread::sleep_for(ringDropStepTime);
    }
    fill_solid(leds, LED_COUNT, CRGB::Green);
    FastLED.show();
    std::this_thread::sleep_for(500ms);

    int rotationCount = 5;
    size_t stepCount = (rotationCount * HIGH_DENSITY_LEDS_PER_RING);
    steady_clock::duration stepTime = animationTime / stepCount;

    steady_clock::time_point startTime = steady_clock::now();

    float phase = 0.0f;
    for (size_t stepCounter = 0; stepCounter < stepCount; stepCounter++)
    {
        size_t bandOffset = 0;
        fill_solid(leds, LED_COUNT, CRGB::Black);
        for (size_t bandIndex = 0; bandIndex < bandSizes.size(); bandIndex++)
        {
            size_t ringSize = bandSizes[bandIndex];
            size_t ringCount = ringCounts[bandIndex];

            bool isReversed = bandIndex % 2;
            size_t ledOnCount = ringSize / 2;

            size_t phaseOffset = static_cast<size_t>(std::round(phase * ringSize)) % ringSize;
            if (isReversed)
                phaseOffset = ringSize - phaseOffset - 1;
            for (size_t ringIndex = 0; ringIndex < ringCount; ringIndex++)
            {
                for (size_t ringOffset = 0; ringOffset < ledOnCount; ringOffset++)
                {
                    size_t ledOffset = (ringOffset + phaseOffset) % ringSize;
                    leds[bandOffset + ledOffset] = CRGB::White;
                }
                bandOffset += ringSize;
            }
        }
        phase += 1.0f / (stepCount / rotationCount);
        FastLED.show();
        std::this_thread::sleep_for(stepTime);
    }
}