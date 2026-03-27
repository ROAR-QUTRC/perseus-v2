#include <freertos/FreeRTOS.h>

#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <optional>
#include <vector>

#include "freertos/idf_additions.h"
#include "hi_can_address.hpp"
#include "hi_can_parameter.hpp"
#include "hi_can_twai.hpp"
#include "portmacro.h"
#include "uart-driver.hpp"
#include "uart-message.hpp"

using namespace hi_can;
using namespace hi_can::addressing;
using namespace hi_can::addressing::power;

const std::chrono::duration message_interval = std::chrono::milliseconds(1000);
std::optional<PacketManager> packet_manager;

constexpr uint8_t BMS_START_BYTE = 0xDD;
constexpr uint8_t BMS_STOP_BYTE = 0x77;
constexpr uint8_t BMS_WRITE_BYTE = 0x5A;
constexpr uint8_t BMS_READ_BYTE = 0xA5;

constexpr uint8_t BMS_HEADER_LENGTH = 3;
constexpr uint8_t BMS_TRAILER_LENGTH = 0;
constexpr uint8_t BMS_FOOTER_LENGTH = 1;

std::optional<UartDriver> uart_driver;
parameters::power::bms::bms_status_t bms_status = {};
parameters::power::bms::bms_temperature_t temperatures = {};
parameters::power::bms::cell_voltage_t voltages = {};

std::function<std::vector<uint8_t>(const raw_uart_message_t)> bms_crc_creator = [](const raw_uart_message_t data)
{
    std::vector<uint8_t> crc;
    uint16_t sum = 0;
    for (uint8_t byte : static_cast<std::vector<uint8_t>>(data))
    {
        sum += byte;
    }
    uint16_t twos_complement_sum = (~sum) + 1;
    crc.emplace_back((twos_complement_sum & 0xFF00) >> 8);
    crc.emplace_back(twos_complement_sum & 0xFF);
    return crc;
};

extern "C" void app_main()
{
    try
    {
        uart_driver.emplace(1, 9600);
    }
    catch (const std::exception& e)
    {
        printf("Error while initialising UART\n");
        return;
    }
    try
    {
        printf("Initialising CAN\n");
        auto& can_interface = TwaiInterface::get_instance(std::make_pair(bsp::CAN_TX_PIN, bsp::CAN_RX_PIN), 0,
                                                          filter_t{
                                                              .address = static_cast<flagged_address_t>(standard_address_t(SYSTEM_ID, battery::SUBSYSTEM_ID, static_cast<uint8_t>(battery::device::BMS))),
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
            .address = static_cast<flagged_address_t>(standard_address_t(SYSTEM_ID, battery::SUBSYSTEM_ID, static_cast<uint8_t>(battery::device::BMS), battery::bms::GROUP_ID, static_cast<uint8_t>(battery::bms::bms_parameter::MOSFET_CONTROL))),
            .mask = MASK_ALL},
        PacketManager::callback_config_t{
            .data_callback = [&](const Packet& packet)
            {
                parameters::power::bms::mosfet_control_t command;
                command.deserialize_data(packet.get_data());
                if (1)
                {
                    printf("I would turn off the BMS right now\n");
                }
                else
                {
                    uart_driver->transmit(static_cast<raw_uart_message_t>(flagged_uart_message_t(std::vector<uint8_t>{
                                                                                                     BMS_START_BYTE,
                                                                                                     BMS_WRITE_BYTE,
                                                                                                     static_cast<uint8_t>(bms_message_command::MOSFET_CONTROL)},
                                                                                                 std::vector<uint8_t>{0x00, static_cast<uint8_t>(command.control_command)}, std::vector<uint8_t>{}, bms_crc_creator, std::vector<uint8_t>{BMS_STOP_BYTE})));
                }
                return; }});
    packet_manager->set_transmission_config(static_cast<flagged_address_t>(standard_address_t(SYSTEM_ID, battery::SUBSYSTEM_ID, static_cast<uint8_t>(battery::device::BMS), battery::bms::GROUP_ID, static_cast<uint8_t>(battery::bms::bms_parameter::STATUS))), PacketManager::transmission_config_t{.generator = [&]()
                                                                                                                                                                                                                                                                                                      {
                                                                                                                                                                                                                                                                                                          printf("Battery status transmitting\n");
                                                                                                                                                                                                                                                                                                          return bms_status.serialize_data();
                                                                                                                                                                                                                                                                                                      },
                                                                                                                                                                                                                                                                                                      .interval = message_interval});
    // packet_manager->set_transmission_config(static_cast<flagged_address_t>(standard_address_t(SYSTEM_ID, battery::SUBSYSTEM_ID, static_cast<uint8_t>(battery::device::BMS), battery::bms::GROUP_ID, static_cast<uint8_t>(battery::bms::bms_parameter::VOLTAGE))), PacketManager::transmission_config_t{.generator = [&]()
    //                                                                                                                                                                                                                                                                                                    {
    //                                                                                                                                                                                                                                                                                                        return voltages.serialize_data();
    //                                                                                                                                                                                                                                                                                                    },
    //                                                                                                                                                                                                                                                                                                    .interval = message_interval});
    //
    packet_manager->set_transmission_config(static_cast<flagged_address_t>(standard_address_t(SYSTEM_ID, battery::SUBSYSTEM_ID, static_cast<uint8_t>(battery::device::BMS), battery::bms::GROUP_ID, static_cast<uint8_t>(battery::bms::bms_parameter::TEMPERATURE))), PacketManager::transmission_config_t{.generator = [&]()
                                                                                                                                                                                                                                                                                                           {
                                                                                                                                                                                                                                                                                                               printf("Temperature transmitting\n");
                                                                                                                                                                                                                                                                                                               return temperatures.serialize_data();
                                                                                                                                                                                                                                                                                                           },
                                                                                                                                                                                                                                                                                                           .interval = message_interval});
    TickType_t last_updated = xTaskGetTickCount();
    while (1)
    {
        printf("Handling packets\n");
        packet_manager->handle();
        if ((xTaskGetTickCount() - last_updated) > 25)
        {
            last_updated = xTaskGetTickCount();
            raw_uart_message_t info_message{};
            printf("Talking to BMS about info\n");
            if (uart_driver->receive_flagged(info_message, 3, 0, bms_crc_creator, 1) == -1)
            {
                printf("Error receiving info message\n");
                bms_status = {};
            }
            else
            {
                printf("Received info message\n");
                std::vector<uint8_t> data = {};
                std::copy(info_message.begin() + BMS_HEADER_LENGTH + DATA_LENGTH_BYTES, info_message.begin() + BMS_HEADER_LENGTH + DATA_LENGTH_BYTES + info_message.at(BMS_HEADER_LENGTH), data.begin());

                bms_status.voltage = data[0] << 8 | data[1];
                bms_status.current = data[2] << 8 | data[3];
                bms_status.current_capacity = data[4] << 8 | data[5];
                bms_status.protection_status = static_cast<parameters::power::bms::_bms_status_t::protection_status_t>(data[16] << 8 | data[17]);
                uint16_t amount_of_NTCs = data[22];

                temperatures = {};
                for (int i = 0; i < amount_of_NTCs; i++)
                {
                    temperatures.emplace_back(data[23 + i] << 8 | data[24 + i]);
                }
            }
            //     raw_uart_message_t voltage_message{};
            //     uart_driver->transmit(static_cast<raw_uart_message_t>(flagged_uart_message_t(std::vector<uint8_t>{BMS_START_BYTE, BMS_READ_BYTE, static_cast<uint8_t>(bms_message_command::VOLTAGE)}, std::vector<uint8_t>{}, std::vector<uint8_t>{}, std::nullopt, std::vector<uint8_t>{BMS_STOP_BYTE})));
            //     if (uart_driver->receive_flagged(voltage_message, 3, 0, bms_crc_creator, 1) == -1)
            //     {
            //         voltages = {};
            //     }
            //     else
            //     {
            //         std::vector<uint8_t> data = {};
            //         std::copy(voltage_message.begin() + BMS_HEADER_LENGTH + DATA_LENGTH_BYTES, voltage_message.begin() + BMS_HEADER_LENGTH + DATA_LENGTH_BYTES + voltage_message.at(BMS_HEADER_LENGTH), data.begin());
            //         uint8_t cells_amount = voltage_message.at(BMS_HEADER_LENGTH);
            //         voltages = {};
            //         for (int i = 0; i < cells_amount; i++)
            //         {
            //             voltages.emplace_back(data.at(2 * i) << 8 | data.at(2 * i + 1));
            //         }
            //     }
        }
        printf("Letting IDLE task run\n");
        vTaskDelay(1);
    }
}
