#include <Arduino.h>
#include <Wire.h>

#include <hi_can_twai.hpp>
#include <optional>

#define SPECTRAL_ADDRESS                 0x00  // TODO: set correct I2C address
#define SPECTRAL_ENABLE_REGISTER_ADDRESS 0x80
#define SPECTRAL_ENABLE_DATA             0b00001011
#define SPECTRAL_DATA_N_L_REGISTER       0x95
#define SPECTRAL_NUM_CHANNELS            18
#define SPECTRAL_READ_BYTES              (SPECTRAL_NUM_CHANNELS * 2)

#define MAGNETOMETER_ADDRESS     0x00  // TODO: set correct I2C address
#define MAGNETOMETER_ENABLE_DATA 0b00110000
#define MAGNETOMETER_ENABLE_XYZ  0b00000111

using namespace hi_can;
using namespace hi_can::addressing;
using namespace std::chrono_literals;

// Sensor data buffers
static uint16_t spectral_data[SPECTRAL_NUM_CHANNELS] = {};
static int16_t magnetometer_data[3] = {};  // x, y, z

std::optional<PacketManager> packet_manager;

constexpr standard_address_t DEVICE_ADDRESS{
    space_resources::SYSTEM_ID,
    space_resources::controller::SUBSYSTEM_ID,
    space_resources::controller::sensing::DEVICE_ID,
};

// Forward declarations
void writeRegister(byte device, byte reg, byte val);
void writeToMagnetometer(byte device, byte val);
bool readSpectral();
bool readMagnetometer();

void setup()
{
    Serial.begin(9600);
    Wire.begin();

    Serial.println("Initialising sensors...");
    writeRegister(SPECTRAL_ADDRESS, SPECTRAL_ENABLE_REGISTER_ADDRESS, 0x01);
    delay(100);
    writeRegister(SPECTRAL_ADDRESS, SPECTRAL_ENABLE_REGISTER_ADDRESS, SPECTRAL_ENABLE_DATA);
    Serial.println("Spectral Sensor Initialised.");

    writeToMagnetometer(MAGNETOMETER_ADDRESS, 0b11110000);
    delay(100);
    Serial.println("Magnetometer Initialised.");

    auto& interface = TwaiInterface::get_instance(std::make_pair(bsp::CAN_TX_PIN, bsp::CAN_RX_PIN), 0,
                                                  filter_t{
                                                      .address = static_cast<flagged_address_t>(DEVICE_ADDRESS),
                                                      .mask = DEVICE_MASK,
                                                  });
    packet_manager.emplace(interface);

    using namespace space_resources::controller::sensing;
    using namespace parameters::space_resources::controller::sensing;

    // Callback for spectral sensor read request
    packet_manager->set_callback(
        filter_t{
            .address = static_cast<flagged_address_t>(standard_address_t{DEVICE_ADDRESS,
                                                                         static_cast<uint8_t>(group::SPECTRAL_SENSOR),
                                                                         static_cast<uint8_t>(spectral_sensor_parameter::START)})},
        {
            .data_callback = [](const Packet&)
            {
                if (!readSpectral())
                    return;

                auto& iface = packet_manager->get_interface();

                iface.transmit(Packet(
                    static_cast<flagged_address_t>(standard_address_t{DEVICE_ADDRESS,
                                                                      static_cast<uint8_t>(group::SPECTRAL_SENSOR),
                                                                      static_cast<uint8_t>(spectral_sensor_parameter::DATA1)}),
                    spectrum_1_t{_spectrum_1_t{spectral_data[0], spectral_data[1], spectral_data[2], spectral_data[3]}}.serialize_data()));

                iface.transmit(Packet(
                    static_cast<flagged_address_t>(standard_address_t{DEVICE_ADDRESS,
                                                                      static_cast<uint8_t>(group::SPECTRAL_SENSOR),
                                                                      static_cast<uint8_t>(spectral_sensor_parameter::DATA2)}),
                    spectrum_2_t{_spectrum_2_t{spectral_data[4], spectral_data[5], spectral_data[6], spectral_data[7]}}.serialize_data()));

                iface.transmit(Packet(
                    static_cast<flagged_address_t>(standard_address_t{DEVICE_ADDRESS,
                                                                      static_cast<uint8_t>(group::SPECTRAL_SENSOR),
                                                                      static_cast<uint8_t>(spectral_sensor_parameter::DATA3)}),
                    spectrum_3_t{_spectrum_3_t{spectral_data[8], spectral_data[9], spectral_data[10], spectral_data[11]}}.serialize_data()));

                iface.transmit(Packet(
                    static_cast<flagged_address_t>(standard_address_t{DEVICE_ADDRESS,
                                                                      static_cast<uint8_t>(group::SPECTRAL_SENSOR),
                                                                      static_cast<uint8_t>(spectral_sensor_parameter::DATA4)}),
                    spectrum_4_t{_spectrum_4_t{spectral_data[12], spectral_data[13], spectral_data[14], spectral_data[15]}}.serialize_data()));

                iface.transmit(Packet(
                    static_cast<flagged_address_t>(standard_address_t{DEVICE_ADDRESS,
                                                                      static_cast<uint8_t>(group::SPECTRAL_SENSOR),
                                                                      static_cast<uint8_t>(spectral_sensor_parameter::DATA5)}),
                    spectrum_5_t{_spectrum_5_t{spectral_data[16], spectral_data[17]}}.serialize_data()));
            },
        });

    // Callback for magnetometer read request
    packet_manager->set_callback(
        filter_t{
            .address = static_cast<flagged_address_t>(standard_address_t{DEVICE_ADDRESS,
                                                                         static_cast<uint8_t>(group::MAGNETOMETER),
                                                                         static_cast<uint8_t>(magnetometer_sensor_parameter::START)})},
        {
            .data_callback = [](const Packet&)
            {
                if (!readMagnetometer())
                    return;

                packet_manager->get_interface().transmit(Packet(
                    static_cast<flagged_address_t>(standard_address_t{DEVICE_ADDRESS,
                                                                      static_cast<uint8_t>(group::MAGNETOMETER),
                                                                      static_cast<uint8_t>(magnetometer_sensor_parameter::XYZ)}),
                    magnet_xyz_t{_magnet_xyz_t{magnetometer_data[0], magnetometer_data[1], magnetometer_data[2]}}.serialize_data()));
            },
        });
}

void writeRegister(byte device, byte reg, byte val)
{
    Wire.beginTransmission(device);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
    delay(100);
}

void writeToMagnetometer(byte device, byte val)
{
    Wire.beginTransmission(device);
    Wire.write(val);
    Wire.endTransmission();
    delay(100);
}

bool readSpectral()
{
    Wire.beginTransmission(SPECTRAL_ADDRESS);
    Wire.write(SPECTRAL_DATA_N_L_REGISTER);
    Wire.endTransmission();

    Wire.requestFrom(SPECTRAL_ADDRESS, SPECTRAL_READ_BYTES);

    if (Wire.available() == SPECTRAL_READ_BYTES)
    {
        for (int i = 0; i < SPECTRAL_NUM_CHANNELS; i++)
        {
            uint16_t low = Wire.read();
            uint16_t high = Wire.read();
            spectral_data[i] = (high << 8) | low;
        }
        return true;
    }
    else
    {
        Serial.println("Error reading spectral sensor");
        return false;
    }
}

bool readMagnetometer()
{
    writeToMagnetometer(MAGNETOMETER_ADDRESS, 0b00111111);
    writeToMagnetometer(MAGNETOMETER_ADDRESS, (MAGNETOMETER_ENABLE_DATA | MAGNETOMETER_ENABLE_XYZ));

    Wire.requestFrom(MAGNETOMETER_ADDRESS, 9);

    if (Wire.available() == 9)
    {
        Wire.read();                                              // status byte
        magnetometer_data[0] = (Wire.read() << 8) | Wire.read();  // x
        magnetometer_data[1] = (Wire.read() << 8) | Wire.read();  // y
        magnetometer_data[2] = (Wire.read() << 8) | Wire.read();  // z
        return true;
    }
    else
    {
        Serial.println("Error reading magnetometer.");
        return false;
    }
}

void loop()
{
    packet_manager->handle();
    delay(1);
}
