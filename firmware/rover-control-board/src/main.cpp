// core
#include <driver/gpio.h>
#include <driver/gptimer.h>
#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_continuous.h>
#include <freertos/FreeRTOS.h>
#include <freertos/timers.h>

// standard libraries
#include <cstdio>

// New canbus
#include <chrono>
#include <hi_can_twai.hpp>
#include <optional>

// rover libs - leftover from Artemis
#include <rover_adc.hpp>
#include <rover_core.hpp>
#include <rover_io.hpp>
#include <rover_log.hpp>
#include <rover_thread.hpp>

#include "power_parameters.hpp"
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

using namespace hi_can;
using namespace hi_can::addressing;
using namespace std::chrono;
using namespace std::chrono_literals;

std::optional<PacketManager> packet_manager;
std::vector<TwaiPowerBusParameterGroup> parameter_groups;

bool button_state = true;

RoverPowerBus spare_bus(hi_can::addressing::power::distribution::rover_control_board::group::SPARE_BUS,
                        CONFIG_PRECHARGE_VOLTAGE, RCB_SPARE_PRE_SWITCH_PIN, RCB_SPARE_MAIN_SWITCH_PIN,
                        ROVER_A2_PIN, ROVER_A1_PIN);
RoverPowerBus drive_bus(hi_can::addressing::power::distribution::rover_control_board::group::DRIVE_BUS,
                        CONFIG_PRECHARGE_VOLTAGE, RCB_DRIVE_PRE_SWITCH_PIN, RCB_DRIVE_MAIN_SWITCH_PIN,
                        ROVER_A4_PIN, ROVER_A3_PIN);
RoverPowerBus compute_bus(hi_can::addressing::power::distribution::rover_control_board::group::COMPUTE_BUS,
                          CONFIG_COMPUTE_PRECHARGE_VOLTAGE, RCB_COMP_PRE_SWITCH_PIN, RCB_COMP_MAIN_SWITCH_PIN,
                          ROVER_A6_PIN, ROVER_A5_PIN);
RoverPowerBus aux_bus(hi_can::addressing::power::distribution::rover_control_board::group::AUX_BUS,
                      CONFIG_AUX_PRECHARGE_VOLTAGE, RCB_AUX_PRE_SWITCH_PIN, RCB_AUX_MAIN_SWITCH_PIN,
                      ROVER_A8_PIN, ROVER_A7_PIN);

const std::vector<std::tuple<std::string, power::distribution::rover_control_board::group, RoverPowerBus&>> BUS_GROUPS = {
    {"compute", power::distribution::rover_control_board::group::COMPUTE_BUS, compute_bus},
    {"drive", power::distribution::rover_control_board::group::DRIVE_BUS, drive_bus},
    {"aux", power::distribution::rover_control_board::group::AUX_BUS, aux_bus},
    {"spare", power::distribution::rover_control_board::group::SPARE_BUS, spare_bus},
};

// New canbus
constexpr standard_address_t RCB_DEVICE_ADDRESS{
    power::SYSTEM_ID,
    power::distribution::SUBSYSTEM_ID,
    power::distribution::rover_control_board::DEVICE_ID,
};

IoDebouncedButton power_button(ROVER_A9_PIN, GPIO_FLOATING, true);

TimerHandle_t timer = nullptr;

uint64_t startup_time = 0;

hi_can::parameters::power::contactor::control_t contactor_data;

void contactor_timer_callback(TimerHandle_t timer);
extern "C" void app_main()  // entry point - ESP-IDF expects C linkage
{
    // the contactor power on MUST be the first thing to occur
    // this ensures that the board will self-power ASAP
    gpio_set_output(RCB_CONTACTOR_PIN);
    gpio_set_output(RCB_POWER_LED_PIN);

    gpio_set_level(RCB_CONTACTOR_PIN, 1);
    // light up button so user knows to release button
    gpio_set_level(RCB_POWER_LED_PIN, 1);

    fflush(stdout);
    vt100ResetTerminal();
    printf("\r\n-----------------------------------------------------------------\r\n");

    core_init();

    try
    {
        auto& can_interface = TwaiInterface::get_instance(std::make_pair(bsp::CAN_TX_PIN, bsp::CAN_RX_PIN), 0,
                                                          filter_t{
                                                              .address = static_cast<flagged_address_t>(RCB_DEVICE_ADDRESS),
                                                              .mask = DEVICE_MASK,
                                                          });
        packet_manager.emplace(can_interface);
    }
    catch (const std::exception& e)
    {
        ERROR("Error \"%s\" while initialising CAN", e.what());
        return;
    }
    try
    {
        parameter_groups.reserve(4);
    }
    catch (const std::exception& e)
    {
        ERROR("Error \"%s\" while reserving space for parameter groups", e.what());
        return;
    }
    for (const auto& [name, id, power_bus] : BUS_GROUPS)
    {
        try
        {
            parameter_groups.emplace_back(power_bus.GetParameterGroup());
            packet_manager->add_group(parameter_groups.back());
            packet_manager->set_transmission_config(
                flagged_address_t(standard_address_t(RCB_DEVICE_ADDRESS, static_cast<uint8_t>(id), static_cast<uint8_t>(hi_can::addressing::power::distribution::rover_control_board::power_bus::parameter::POWER_STATUS))),
                power_bus.GetTransmissionConfig());
        }
        catch (const std::exception& e)
        {
            ERROR("Error \"%s\" while setting parameter groups for %s", e.what(), name.c_str());
            return;
        }
    }

    timer = timer_create(contactor_timer_callback, 1000);

    packet_manager->set_callback(
        filter_t{static_cast<flagged_address_t>(
            standard_address_t{power::distribution::rover_control_board::DEVICE_ID,
                               static_cast<uint8_t>(power::distribution::rover_control_board::group::CONTACTOR),
                               static_cast<uint8_t>(power::distribution::rover_control_board::contactor::parameter::SHUTDOWN)})},
        PacketManager::callback_config_t{
            .data_callback = [&](hi_can::Packet packet)
            {
                hi_can::parameters::power::contactor::control_t data;
                data.deserialize_data(packet.get_data());
                if (data.immediate_shutdown)
                {
                    xTimerReset(timer, 0);
                    WARN("Performing immediate shutdown in 100ms!");
                    fflush(stdout);
                    DELAY_MS(100);
                    gpio_set_level(RCB_CONTACTOR_PIN, 0);
                }
                else
                {
                    contactor_data.immediate_shutdown = data.immediate_shutdown;
                    contactor_data.shutdown_timer = data.shutdown_timer;
                }
            },
        });

    startup_time = core_get_uptime();
    power_button.clear_has_hold();
    power_button.clear_has_press();
    INFO("Starting core 1 main loop");
    thread_create([](void* args)
                  { while(true) loop(args); },
                  CORE_1, PRIORITY_HIGH, "loop", 8096);
}

void loop(void* args)
{
    static bool blink_state = true;
    static uint64_t last_blink_toggle = 0;
    static uint64_t last_press = 0;
    static int last_repeat_count = 0;

    packet_manager->handle();

    if ((core_get_uptime() - last_blink_toggle) >= 300)
    {
        blink_state = !blink_state;
        last_blink_toggle = core_get_uptime();
    }

    if (!compute_bus.is_bus_on())
    {
        gpio_set_level(RCB_POWER_LED_PIN, blink_state);
    }
    else
    {
        gpio_set_level(RCB_POWER_LED_PIN, true);
    }
    power_button.handle();
    const uint64_t now = core_get_uptime();
    if (power_button.has_hold())
    {
        if (compute_bus.is_bus_on())
        {
            INFO("Turning off buses");
            drive_bus.set_bus_state(false);
            aux_bus.set_bus_state(false);
            spare_bus.set_bus_state(false);
            compute_bus.set_bus_state(false);
        }
        else
        {
            INFO("Turning off contactor in 100ms");
            fflush(stdout);
            DELAY_MS(100);
            gpio_set_level(RCB_CONTACTOR_PIN, 0);
        }
    }
    else if (power_button.has_press())
    {
        last_press = now;
        last_repeat_count = power_button.get_repeat_press_count();
        gpio_set_level(RCB_CONTACTOR_PIN, 1);
    }

    if (last_press && ((now - last_press) > 500))
    {
        last_press = 0;
        RoverPowerBus* bus = nullptr;
        std::string bus_name = "#";
        bus_name += (last_repeat_count + 2);
        switch (last_repeat_count)
        {
        default:
        case 0:
            if (!compute_bus.is_bus_on())
            {
                bus = &compute_bus;
                bus_name = "Compute";
            }
            else
            {
                bus = &drive_bus;
                bus_name = "Drive";
            }
            break;
        case 1:
            bus = &aux_bus;
            bus_name = "Aux";
            break;
        case 2:
            bus = &spare_bus;
            bus_name = "Spare";
            break;
        }

        if (bus->is_bus_on())
        {
            INFO("%s bus turning off", bus_name.c_str());
            bus->set_bus_state(false);
        }
        else
        {
            INFO("%s bus turning on", bus_name.c_str());
            bus->clear_error();
            bus->set_bus_state(true);
        }
    }

    spare_bus.handle();
    drive_bus.handle();
    compute_bus.handle();
    aux_bus.handle();

    // let idle task run
    vTaskDelay(1);
}

static void contactor_timer_callback(TimerHandle_t timer)
{
    if (contactor_data.shutdown_timer)
    {
        contactor_data.shutdown_timer--;
        if (contactor_data.shutdown_timer == 0)
        {
            WARN("Performing scheduled shutdown in 100ms!");
            fflush(stdout);
            DELAY_MS(100);
            gpio_set_level(RCB_CONTACTOR_PIN, 0);  // shut down whole rover
        }
    }
}
