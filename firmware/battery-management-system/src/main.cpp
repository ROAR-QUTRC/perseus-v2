#include "main.hpp"

#include <Arduino.h>

#include <hi_can_twai.hpp>

#include "bq769x2.hpp"

void setup()
{
    bsp::initI2C();
    Serial.begin(115200);
    hi_can::TwaiInterface::getInstance();
    try
    {
        // bq76942 bq;
        // bq.writeSubcommand(bq76942::cmd_only_subcommand::RESET);
    }
    catch (std::exception& e)
    {
        printf(std::format("Error resetting BMS: {}\n", e.what()).c_str());
    }
}

void loop()
{
    using namespace hi_can;

    auto& interface = TwaiInterface::getInstance();
    try
    {
        interface.transmit(Packet{});
    }
    catch (const std::exception& e)
    {
        printf(std::format("{}\n", e.what()).c_str());
    }
    // scan for I2C devices
    try
    {
        // bq76942 bq;
        // // bq.writeSubcommand(bq76942::cmd_only_subcommand::RESET);
        // // delay(100);
        // bq.writeSubcommand<uint16_t>(bq76942::data_register::VCELL_MODE, 0x011F);
        // bq.writeSubcommand<uint8_t>(bq76942::data_register::DA_CONFIGURATION, 0b00000010);
        // // for (int i = 0; i < 16; i++)
        // // {
        // //     printf(std::format("Voltage {}: {}\n", i + 1, bq.readDirect<int16_t>(static_cast<uint8_t>(bq76942::direct_command::CELL_1_VOLTAGE) + (2 * i))).c_str());
        // //     // printf(std::format("Voltage 2: {}\n", bq.readDirect<int16_t>(bq76942::direct_command::CELL_2_VOLTAGE)).c_str());
        // //     // printf(std::format("Voltage 3: {}\n", bq.readDirect<int16_t>(bq76942::direct_command::CELL_3_VOLTAGE)).c_str());
        // // }
        // printf(std::format("Voltage 1: {}\n", bq.readDirect<int16_t>(bq76942::direct_command::CELL_1_VOLTAGE)).c_str());
        // printf(std::format("Voltage 2: {}\n", bq.readDirect<int16_t>(bq76942::direct_command::CELL_2_VOLTAGE)).c_str());
        // printf(std::format("Voltage 3: {}\n", bq.readDirect<int16_t>(bq76942::direct_command::CELL_3_VOLTAGE)).c_str());
        // printf(std::format("Voltage 4: {}\n", bq.readDirect<int16_t>(bq76942::direct_command::CELL_4_VOLTAGE)).c_str());
        // printf(std::format("Voltage 5: {}\n", bq.readDirect<int16_t>(bq76942::direct_command::CELL_5_VOLTAGE)).c_str());
        // printf(std::format("Voltage 6: {}\n", bq.readDirect<int16_t>(bq76942::direct_command::CELL_6_VOLTAGE)).c_str());
        // printf(std::format("Voltage 7: {}\n", bq.readDirect<int16_t>(bq76942::direct_command::CELL_7_VOLTAGE)).c_str());
        // printf(std::format("Voltage 8: {}\n", bq.readDirect<int16_t>(bq76942::direct_command::CELL_8_VOLTAGE)).c_str());
        // printf(std::format("Voltage 9: {}\n", bq.readDirect<int16_t>(bq76942::direct_command::CELL_9_VOLTAGE)).c_str());
        // printf(std::format("Voltage 10: {}\n", bq.readDirect<int16_t>(bq76942::direct_command::CELL_10_VOLTAGE)).c_str());
        // printf(std::format("Pack Voltage: {}\n", bq.readDirect<int16_t>(bq76942::direct_command::PACK_PIN_VOLTAGE)).c_str());
        // printf(std::format("Stack Voltage: {}\n", bq.readDirect<int16_t>(bq76942::direct_command::STACK_VOLTAGE)).c_str());
        // printf(std::format("LD Voltage: {}\n", bq.readDirect<int16_t>(bq76942::direct_command::LD_PIN_VOLTAGE)).c_str());
        // printf(std::format("Int Temp: {:.1f}C\n", (bq.readDirect<int16_t>(bq76942::direct_command::INT_TEMPERATURE) / 10.0) - 273.15).c_str());
        // printf(std::format("FET stat: {:#04x}\n", bq.readDirect<int16_t>(bq76942::direct_command::FET_STATUS)).c_str());
        // printf(std::format("Safety Alert A: {:#04x}\n", bq.readDirect<int16_t>(bq76942::direct_command::SAFETY_ALERT_A)).c_str());
        // printf(std::format("Safety Alert B: {:#04x}\n", bq.readDirect<int16_t>(bq76942::direct_command::SAFETY_ALERT_B)).c_str());
        // printf(std::format("Safety Alert C: {:#04x}\n", bq.readDirect<int16_t>(bq76942::direct_command::SAFETY_ALERT_C)).c_str());
        // printf(std::format("Manu stat: {:#06x}\n", bq.readSubcommand<uint16_t>(bq76942::subcommand::MANUFACTURING_STATUS)).c_str());

        // static int fetCounter = 0;
        // fetCounter++;
        // printf(std::format("FET counter: {}\n", fetCounter).c_str());
        // if (fetCounter == 15)
        // {
        //     // bq.writeSubcommand(bq76942::cmd_only_subcommand::ALL_FETS_ON);
        //     // bq.writeSubcommand(bq76942::cmd_only_subcommand::PCHG_TEST);
        //     // bq.writeSubcommand(bq76942::cmd_only_subcommand::PDSG_TEST);
        // }
        // else if (fetCounter == 19)
        // {
        //     // bq.writeSubcommand(bq76942::cmd_only_subcommand::PCHG_TEST);
        //     // bq.writeSubcommand(bq76942::cmd_only_subcommand::PDSG_TEST);
        // }
        // if (fetCounter > 20)
        // {
        //     // printf("Shutting down\n");
        //     // bq.shutdown();
        // }
        // // if (fetCounter < 2)
        // // {
        // //     // bq.writeSubcommand(bq76942::cmd_only_subcomand::CHG_TEST);
        // //     printf("FETs on\n");
        // // }
        // // else
        // // {
        // //     // bq.writeSubcommand(bq76942::cmd_only_subcommand::ALL_FETS_OFF);
        // //     printf("FETs off\n");
        // // }
        // // if (fetCounter > 4)
        // // {
        // //     fetCounter = 0;
        // // }

        // // vTaskDelay(5);
        // // bq.writeRegister<uint16_t>(bq76942::register_t::SUBCOMMAND, 0x0001);
        // // printf("reading...\n");
        // // vTaskDelay(10);
        // // bq.readRegister(bq76942::register_t::TRANSFER_BUF, 34);
        // auto data = bq.readSubcommand<uint16_t>(0x0001);
        // printf("Got: %x\n", data);
        // bq.writeSubcommand(bq76942::cmd_only_subcommand::FUSE_TOGGLE);
    }
    catch (const std::exception& e)
    {
        printf(std::format("err: {}\n", e.what()).c_str());
    }
    // printf("Scanning I2C bus...\n");
    // for (uint8_t addr = 0; addr < 128; addr++)
    // {
    //     Wire.beginTransmission(addr);
    //     if (Wire.endTransmission() == 0)
    //     {
    //         printf("Found device at address 0x%02x\n", addr);
    //     }
    // }
    vTaskDelay(1000);
    printf("Test\n");
}