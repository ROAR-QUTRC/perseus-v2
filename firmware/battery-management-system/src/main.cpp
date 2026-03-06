#include <chrono>
#include <cstdint>
#include <exception>
#include <optional>
#include <vector>

#include "hi_can_address.hpp"
#include "hi_can_parameter.hpp"
#include "hi_can_twai.hpp"
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

UartDriver uart_driver(2, 9600);

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
        auto& can_interface = TwaiInterface::get_instance(std::make_pair(bsp::CAN_TX_PIN, bsp::CAN_RX_PIN), 0,
                                                          filter_t{
                                                              .address = static_cast<flagged_address_t>(standard_address_t(SYSTEM_ID, battery::SUBSYSTEM_ID)),
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
                uart_driver.transmit(static_cast<raw_uart_message_t>(flagged_uart_message_t(std::vector<uint8_t>{
                                                                                                BMS_START_BYTE,
                                                                                                BMS_WRITE_BYTE,
                                                                                                static_cast<uint8_t>(battery::bms::bms_parameter::MOSFET_CONTROL)},
                                                                                            std::vector<uint8_t>{0x00, static_cast<uint8_t>(command.control_command)}, std::vector<uint8_t>{}, bms_crc_creator, std::vector<uint8_t>{BMS_STOP_BYTE})));
                return;
            }});
    packet_manager->set_transmission_config(static_cast<flagged_address_t>(standard_address_t(SYSTEM_ID, battery::SUBSYSTEM_ID, static_cast<uint8_t>(battery::device::BMS), battery::bms::GROUP_ID, static_cast<uint8_t>(battery::bms::bms_parameter::INFO))), PacketManager::transmission_config_t{.generator = [&]()
                                                                                                                                                                                                                                                                                                    {
                                                                                                                                                                                                                                                                                                        raw_uart_message_t received_message{};
                                                                                                                                                                                                                                                                                                        uart_driver.transmit(static_cast<raw_uart_message_t>(flagged_uart_message_t(std::vector<uint8_t>{BMS_START_BYTE, BMS_READ_BYTE, static_cast<uint8_t>(battery::bms::bms_parameter::INFO)}, std::vector<uint8_t>{}, std::vector<uint8_t>{}, std::nullopt, std::vector<uint8_t>{BMS_STOP_BYTE})));
                                                                                                                                                                                                                                                                                                        if (uart_driver.receive_flagged(received_message, 3, 0, bms_crc_creator, 1) == -1)
                                                                                                                                                                                                                                                                                                        {
                                                                                                                                                                                                                                                                                                            return std::vector<uint8_t>{};
                                                                                                                                                                                                                                                                                                        }
                                                                                                                                                                                                                                                                                                        std::vector<uint8_t> data = {};
                                                                                                                                                                                                                                                                                                        std::copy(received_message.begin() + BMS_HEADER_LENGTH + DATA_LENGTH_BYTES, received_message.begin() + BMS_HEADER_LENGTH + DATA_LENGTH_BYTES + received_message.at(BMS_HEADER_LENGTH), data.begin());
                                                                                                                                                                                                                                                                                                        parameters::power::bms::bms_information_t information = {};
                                                                                                                                                                                                                                                                                                        information.voltage = data[0] << 8 | data[1];
                                                                                                                                                                                                                                                                                                        information.current = data[2] << 8 | data[3];
                                                                                                                                                                                                                                                                                                        information.current_capacity = data[4] << 8 | data[5];
                                                                                                                                                                                                                                                                                                        information.nominal_capacity = data[6] << 8 | data[7];
                                                                                                                                                                                                                                                                                                        information.cycles = data[8] << 8 | data[9];
                                                                                                                                                                                                                                                                                                        // 10-11 is the product data
                                                                                                                                                                                                                                                                                                        information.balance_status = data[14] << 24 | data[15] << 16 | data[12] << 8 | data[13];
                                                                                                                                                                                                                                                                                                        information.protection_status = static_cast<parameters::power::bms::_bms_information::protection_status_t>(data[16] << 8 | data[17]);
                                                                                                                                                                                                                                                                                                        // 18 is the BMS version
                                                                                                                                                                                                                                                                                                        information.percent_remaining = data[19];
                                                                                                                                                                                                                                                                                                        information.mosfet_status = static_cast<parameters::power::bms::_bms_information::mosfet_status_t>(data[20]);
                                                                                                                                                                                                                                                                                                        information.amount_of_cells = data[21];
                                                                                                                                                                                                                                                                                                        information.amount_of_NTCs = data[22];
                                                                                                                                                                                                                                                                                                        information.temperatures = {};
                                                                                                                                                                                                                                                                                                        for (int i = 0; i < information.amount_of_NTCs; i++)
                                                                                                                                                                                                                                                                                                        {
                                                                                                                                                                                                                                                                                                            information.temperatures.emplace_back(data[23 + i] << 8 | data[24 + i]);
                                                                                                                                                                                                                                                                                                        }
                                                                                                                                                                                                                                                                                                        return information.serialize_data();
                                                                                                                                                                                                                                                                                                    },
                                                                                                                                                                                                                                                                                                    .interval = message_interval});
    packet_manager->set_transmission_config(static_cast<flagged_address_t>(standard_address_t(SYSTEM_ID, battery::SUBSYSTEM_ID, static_cast<uint8_t>(battery::device::BMS), battery::bms::GROUP_ID, static_cast<uint8_t>(battery::bms::bms_parameter::VOLTAGE))), PacketManager::transmission_config_t{.generator = [&]()
                                                                                                                                                                                                                                                                                                       {
                                                                                                                                                                                                                                                                                                           raw_uart_message_t received_message{};
                                                                                                                                                                                                                                                                                                           uart_driver.transmit(static_cast<raw_uart_message_t>(flagged_uart_message_t(std::vector<uint8_t>{BMS_START_BYTE, BMS_READ_BYTE, static_cast<uint8_t>(battery::bms::bms_parameter::INFO)}, std::vector<uint8_t>{}, std::vector<uint8_t>{}, std::nullopt, std::vector<uint8_t>{BMS_STOP_BYTE})));
                                                                                                                                                                                                                                                                                                           if (uart_driver.receive_flagged(received_message, 3, 0, bms_crc_creator, 1) == -1)
                                                                                                                                                                                                                                                                                                           {
                                                                                                                                                                                                                                                                                                               return std::vector<uint8_t>{};
                                                                                                                                                                                                                                                                                                           }
                                                                                                                                                                                                                                                                                                           std::vector<uint8_t> data = {};
                                                                                                                                                                                                                                                                                                           std::copy(received_message.begin() + BMS_HEADER_LENGTH + DATA_LENGTH_BYTES, received_message.begin() + BMS_HEADER_LENGTH + DATA_LENGTH_BYTES + received_message.at(BMS_HEADER_LENGTH), data.begin());
                                                                                                                                                                                                                                                                                                           uint8_t data_length = received_message.at(BMS_HEADER_LENGTH);
                                                                                                                                                                                                                                                                                                           parameters::power::bms::cell_voltage_t received_voltage = {};
                                                                                                                                                                                                                                                                                                           for (int i = 0; i < data_length; i++)
                                                                                                                                                                                                                                                                                                           {
                                                                                                                                                                                                                                                                                                               received_voltage.emplace_back(data.at(2 * i) << 8 | data.at(2 * i + 1));
                                                                                                                                                                                                                                                                                                           }
                                                                                                                                                                                                                                                                                                           return received_voltage.serialize_data();
                                                                                                                                                                                                                                                                                                       },
                                                                                                                                                                                                                                                                                                       .interval = message_interval});

    while (1)
    {
        packet_manager->handle();
    }
}
