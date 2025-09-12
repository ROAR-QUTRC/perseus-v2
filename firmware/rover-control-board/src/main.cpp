#include "main.hpp"

// core
#include <driver/gpio.h>
#include <driver/gptimer.h>
#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_continuous.h>
#include <freertos/FreeRTOS.h>
#include <freertos/timers.h>

// standard libraries
#include <cstdio>

// Old canbus
/*
#include <canlib.hpp>
*/
#include <canlib_power.hpp>

// New canbus
#include <chrono>
#include <hi_can_twai.hpp>
#include <optional>

// rover libs
#include <rover_adc.hpp>
#include <rover_core.hpp>
#include <rover_io.hpp>
#include <rover_log.hpp>
#include <rover_thread.hpp>

#include "rcb.hpp"
#include "rover_debounce.hpp"

#define RCB_POWER_LED_PIN GPIO_NUM_12
#define RCB_CONTACTOR_PIN GPIO_NUM_13

#define RCB_SPARE_PRE_SWITCH_PIN  GPIO_NUM_41
#define RCB_SPARE_MAIN_SWITCH_PIN GPIO_NUM_40

#define RCB_DRIVE_PRE_SWITCH_PIN  GPIO_NUM_39
#define RCB_DRIVE_MAIN_SWITCH_PIN GPIO_NUM_38

#define RCB_COMP_PRE_SWITCH_PIN  GPIO_NUM_37
#define RCB_COMP_MAIN_SWITCH_PIN GPIO_NUM_45

#define RCB_AUX_PRE_SWITCH_PIN  GPIO_NUM_48
#define RCB_AUX_MAIN_SWITCH_PIN GPIO_NUM_47

void loop(void* args);
/* Old canbus
CanlibCommonParameterGroup commonParams(CANLIB_NO_HANDLER, coreRestart);
*/
// New canbus
using namespace hi_can;
using namespace hi_can::addressing;
using namespace std::chrono;
using namespace std::chrono_literals;

std::optional<PacketManager> packetManager;

bool btnState = true;
RoverPowerBus spareBus(CANLIB_GROUP_POWER_SPARE_BUS, CONFIG_PRECHARGE_VOLTAGE,
                       RCB_SPARE_PRE_SWITCH_PIN, RCB_SPARE_MAIN_SWITCH_PIN,
                       ROVER_A2_PIN, ROVER_A1_PIN);
RoverPowerBus driveBus(CANLIB_GROUP_POWER_DRIVE_BUS,
                       CONFIG_PRECHARGE_VOLTAGE, RCB_DRIVE_PRE_SWITCH_PIN, RCB_DRIVE_MAIN_SWITCH_PIN,
                       ROVER_A4_PIN, ROVER_A3_PIN);
RoverPowerBus compBus(CANLIB_GROUP_POWER_COMPUTE_BUS,
                      CONFIG_COMPUTE_PRECHARGE_VOLTAGE, RCB_COMP_PRE_SWITCH_PIN, RCB_COMP_MAIN_SWITCH_PIN,
                      ROVER_A6_PIN, ROVER_A5_PIN);
RoverPowerBus auxBus(CANLIB_GROUP_POWER_AUX_BUS,
                     CONFIG_AUX_PRECHARGE_VOLTAGE, RCB_AUX_PRE_SWITCH_PIN, RCB_AUX_MAIN_SWITCH_PIN,
                     ROVER_A8_PIN, ROVER_A7_PIN);

// New canbus
constexpr standard_address_t RCB_DEVICE_ADDRESS{
    power::SYSTEM_ID,
    power::distribution::SUBSYSTEM_ID,
    power::distribution::rover_control_board::DEVICE_ID,
};

IoDebouncedButton powerButton(ROVER_A9_PIN, GPIO_FLOATING, true);

TimerHandle_t timer = nullptr;

uint64_t startupTime = 0;

canlib_power_contactor_data contactorData = {
    .immediate_shutdown = false,
    .shutdown_timer = 0};
void contactorTimerCb(TimerHandle_t timer);
extern "C" void app_main()  // entry point - ESP-IDF expects C linkage
{
    // the contactor power on MUST be the first thing to occur
    // this ensures that the board will self-power ASAP
    ioConfigOutput(RCB_CONTACTOR_PIN);
    ioConfigOutput(RCB_POWER_LED_PIN);

    gpio_set_level(RCB_CONTACTOR_PIN, 1);
    // light up button so user knows to release button
    gpio_set_level(RCB_POWER_LED_PIN, 1);

    fflush(stdout);
    vt100ResetTerminal();
    printf("\r\n-----------------------------------------------------------------\r\n");

    coreInit();

    /*Old canbus
    canlibInit({CANLIB_SYSTEM_POWER,
                CANLIB_SUBSYSTEM_POWER_CTRL,
                CANLIB_DEVICE_ROVER_CONTROL_BOARD});

    commonParams.setStatus(CANLIB_STATE_NORMAL);
    */
    // New canbus
    auto& interface = TwaiInterface::getInstance(std::make_pair(bsp::CAN_TX_PIN, bsp::CAN_RX_PIN), 0,
                                                 filter_t{
                                                     .address = static_cast<flagged_address_t>(RCB_DEVICE_ADDRESS),
                                                     .mask = DEVICE_MASK,
                                                 });
    packetManager.emplace(interface);

    timer = timerCreate(contactorTimerCb, 1000);
    /*
    canlibAddParameter(
        canlib_parameter_description{
            .address =
                {
                    .group = CANLIB_GROUP_POWER_CONTACTOR,
                    .parameter = CANLIB_PARAM_POWER_CONTACTOR,
                },
            .writable = true,
            .data_change_handler = [&](auto addr)
            {
                xTimerReset(timer, 0);
                canlibGetParameter(addr, contactorData);
                if (contactorData.immediate_shutdown)
                {
                    WARN("Performing immediate shutdown in 100ms!");
                    fflush(stdout);
                    DELAY_MS(100);
                    gpio_set_level(RCB_CONTACTOR_PIN, 0);
                }
            }},
        contactorData);
    */
    packetManager->setCallback(
        filter_t{static_cast<flagged_address_t>(
            standard_address_t{power::distribution::rover_control_board::DEVICE_ID,
                               static_cast<uint8_t>(power::distribution::rover_control_board::group::COMPUTE_BUS),
                               static_cast<uint8_t>(power::distribution::rover_control_board::contactor::parameter::SHUTDOWN)})},
        {
            .dataCallback = [&](auto addr)
            {
                xTimerReset(timer, 0);
                WARN("Performing immediate shutdown in 100ms!");
                fflush(stdout);
                DELAY_MS(100);
                gpio_set_level(RCB_CONTACTOR_PIN, 0);
            },
        });

    startupTime = coreGetUptime();
    powerButton.clearHasHold();
    powerButton.clearHasPress();
    INFO("Starting core 1 main loop");
    threadCreate([](void* args)
                 { while(true) loop(args); },
                 CORE_1, PRIORITY_HIGH, "loop", 8096);
}

void loop(void* args)
{
    static bool blinkState = true;
    static uint64_t lastBlinkToggle = 0;
    static uint64_t lastPress = 0;
    static int lastRepeatCount = 0;

    canlibHandle();

    if ((coreGetUptime() - lastBlinkToggle) >= 300)
    {
        blinkState = !blinkState;
        lastBlinkToggle = coreGetUptime();
    }

    if (!compBus.isBusOn())
    {
        gpio_set_level(RCB_POWER_LED_PIN, blinkState);
    }
    else
        gpio_set_level(RCB_POWER_LED_PIN, true);

    powerButton.handle();
    const uint64_t now = coreGetUptime();
    if (powerButton.hasHold())
    {
        if (compBus.isBusOn())
        {
            INFO("Turning off buses");
            driveBus.setBusOn(false);
            auxBus.setBusOn(false);
            spareBus.setBusOn(false);
            compBus.setBusOn(false);
        }
        else
        {
            INFO("Turning off contactor in 100ms");
            fflush(stdout);
            DELAY_MS(100);
            gpio_set_level(RCB_CONTACTOR_PIN, 0);
        }
    }
    else if (powerButton.hasPress())
    {
        lastPress = now;
        lastRepeatCount = powerButton.getRepeatPressCount();
        gpio_set_level(RCB_CONTACTOR_PIN, 1);
    }

    if (lastPress && ((now - lastPress) > 500))
    {
        lastPress = 0;
        RoverPowerBus* bus = nullptr;
        std::string busName = "#";
        busName += (lastRepeatCount + 2);
        switch (lastRepeatCount)
        {
        default:
        case 0:
            if (!compBus.isBusOn())
            {
                bus = &compBus;
                busName = "Compute";
            }
            else
            {
                bus = &driveBus;
                busName = "Drive";
            }
            break;
        case 1:
            bus = &auxBus;
            busName = "Aux";
            break;
        case 2:
            bus = &spareBus;
            busName = "Spare";
            break;
        }

        if (bus->isBusOn())
        {
            INFO("%s bus turning off", busName.c_str());
            bus->setBusOn(false);
        }
        else
        {
            INFO("%s bus turning on", busName.c_str());
            bus->clearError();
            bus->setBusOn(true);
        }
    }

    spareBus.handle();
    driveBus.handle();
    compBus.handle();
    auxBus.handle();

    // let idle task run
    vTaskDelay(1);
}

void contactorTimerCb(TimerHandle_t timer)
{
    if (contactorData.shutdown_timer)
    {
        contactorData.shutdown_timer--;
        if (contactorData.shutdown_timer == 0)
        {
            WARN("Performing scheduled shutdown in 100ms!");
            fflush(stdout);
            DELAY_MS(100);
            gpio_set_level(RCB_CONTACTOR_PIN, 0);  // shut down whole rover}
        }
    }
}