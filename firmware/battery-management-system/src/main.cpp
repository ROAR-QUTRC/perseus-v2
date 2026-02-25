#include <freertos/FreeRTOS.h>

#include <hi_can_twai.hpp>

#include "bms_uart.hpp"
#include "freertos/task.h"
#include "freertos/timers.h"

using namespace hi_can;
using namespace hi_can::addressing;

const int STACK_SIZE = 8096;
const int LOOP_PRIORITY = 5;
const int LOOP_CORE = 1;
std::optional<PacketManager> packet_manager;
BmsUartDriver uart_driver(UART_NUM_2);

void loop();
extern "C" void app_main()
{
    try
    {
        auto& can_interface = TwaiInterface::get_instance(std::make_pair(bsp::CAN_TX_PIN, bsp::CAN_RX_PIN), 0,
                                                          filter_t{
                                                              .address = static_cast<flagged_address_t>(standard_address_t(power::SYSTEM_ID, power::battery::SUBSYSTEM_ID)),
                                                              .mask = DEVICE_MASK,
                                                          });
        packet_manager.emplace(can_interface);
    }
    catch (const std::exception& e)
    {
        printf("Error \"%s\" while initialising CAN", e.what());
        return;
    }
    packet_manager->set_callback(
        filter_t{
            .address = static_cast<flagged_address_t>(standard_address_t(power::SYSTEM_ID, power::battery::SUBSYSTEM_ID, static_cast<uint8_t>(power::battery::device::BMS), 0x00, static_cast<uint8_t>(power::battery::bms_parameter::MOSFET_CONTROL))),
            .mask = MASK_ALL},
        PacketManager::callback_config_t{
            .data_callback = [&](const Packet& packet)
            {
                parameters::power::bms::mosfet_control command;
                command.deserialize_data(packet.get_data());
                // TODO: Send mosfet command to the bms
                return;
            }});
    packet_manager->set_transmission_config(static_cast<flagged_address_t>(standard_address_t(power::SYSTEM_ID, power::battery::SUBSYSTEM_ID, static_cast<uint8_t>(power::battery::device::BMS), 0x00, static_cast<uint8_t>(power::battery::bms_parameter::INFO))), PacketManager::transmission_config_t{.generator = [&]()
                                                                                                                                                                                                                                                                                                         {
                                                                                                                                                                                                                                                                                                             // TODO: Read information data from UART
                                                                                                                                                                                                                                                                                                             std::vector<uint8_t> data{};
                                                                                                                                                                                                                                                                                                             BmsUartMessage information_message(BmsUartMessage::command_byte_t::INFORMATION, data);
                                                                                                                                                                                                                                                                                                             if (uart_driver.transmit(information_message))
                                                                                                                                                                                                                                                                                                             {
                                                                                                                                                                                                                                                                                                                 printf("Error while transmitting information request message!!!\n");
                                                                                                                                                                                                                                                                                                             }
                                                                                                                                                                                                                                                                                                             return std::vector<uint8_t>{0};
                                                                                                                                                                                                                                                                                                         },
                                                                                                                                                                                                                                                                                                         .interval = uart_driver.get_information_interval()});
    packet_manager->set_transmission_config(static_cast<flagged_address_t>(standard_address_t(power::SYSTEM_ID, power::battery::SUBSYSTEM_ID, static_cast<uint8_t>(power::battery::device::BMS), 0x00, static_cast<uint8_t>(power::battery::bms_parameter::VOLTAGE))), PacketManager::transmission_config_t{.generator = [&]()
                                                                                                                                                                                                                                                                                                            {
                                                                                                                                                                                                                                                                                                                std::vector<uint8_t> data{};
                                                                                                                                                                                                                                                                                                                BmsUartMessage voltage_message(BmsUartMessage::command_byte_t::VOLTAGE, data);
                                                                                                                                                                                                                                                                                                                if (uart_driver.transmit(voltage_message))
                                                                                                                                                                                                                                                                                                                {
                                                                                                                                                                                                                                                                                                                    printf("Error while transmitting voltage request message!!!\n");
                                                                                                                                                                                                                                                                                                                }
                                                                                                                                                                                                                                                                                                                // TODO: Read voltage data from UART
                                                                                                                                                                                                                                                                                                                return std::vector<uint8_t>{0};
                                                                                                                                                                                                                                                                                                            },
                                                                                                                                                                                                                                                                                                            .interval = uart_driver.get_voltage_interval()});

    xTaskCreatePinnedToCore([](void*)
                            {
    while(true) loop(); }, "loop", STACK_SIZE, NULL, LOOP_PRIORITY, NULL, LOOP_CORE);
}

void loop()
{
    packet_manager->handle();
}