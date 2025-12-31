#include <Wire.h>

#define spectral_address                 0x00  // Change this!
#define spectral_enable_register_address 0x80
#define spectral_enable_data             0b00001011
#define spectral_data_N_L_register       0x95

#define magnetometer_address     0x00  // Change this!
#define magnetometer_enable_data 0b00110000
// #define magnetometer_enable_txyz 0b00001111
#define magnetometer_enable_xyz 0b00000111

using namespace hi_can;

PacketManager packetManager;

void setup()
{
    Serial.begin(9600);
    Wire.begin();

    Serial.println("Initialising sensors...");
    writeRegister(spectral_address, spectral_enable_register_address, 0x01);
    delay(100);
    writeRegister(spectral_address, spectral_enable_register_address, spectral_enable_data)
        Serial.println("Spectral Sensor Initialised.");

    writeToMagnetometer(magnetometer_address, 0b11110000);
    delay(100);
    Serial.println("Magnetometer Initialised.");

    auto& sensors_interface = TwaiInterface::getInstance(std::make_pair(bsp::CAN_TX_PIN, bsp::CAN_RX_PIN), 0,
                                                         filter_t{
                                                             .address = static_cast<flagged_address_t>(standard_address_t(space_resources::SYSTEM_ID, space_resources::controller::SUBSYSTEM_ID, space_resources::controller::sensing::DEVICE_ID)),
                                                             .mask = DEVICE_MASK,
                                                         });
    packetManager.emplace(sensors_interface);

    // Callback for the spectral sensor
    packetManager->setCallback(
        addressing::filter_t{
            .address = flagged_address_t(standard_address_t(space_resources::SYSTEM_ID, space_resources::controller::SUBSYSTEM_ID, space_resources::controller::sensing::DEVICE_ID, space_resources::controller::sensing::group::SPECTRAL_SENSOR, space_resources::controller::sensing::spectral_sensor_parameter::START))},
        callback_config_t{
            .datacallback = [&](const Packet)
            {
                start_reading_spectral();
                uint16_t* data = get_spectral_readings();
                // uint16_t data[18] = get_reading();
                Packet data_1(flagged_address_t(standard_address_t(space_resources::SYSTEM_ID, space_resources::controller::SUBSYSTEM_ID, space_resources::controller::sensing::DEVICE_ID, space_resources::controller::sensing::group::SPECTRAL_SENSOR, space_resources::controller::sensing::spectral_sensor_parameter::DATA1)), parameters::space_resources::controller::sensing::spectrum_1_t {
                        .blue_450nm = data[0];
                        .green_555nm = data[1];
                        .orange_600nm = data[2];
                        .nir_855nm = data[3]; }.serializeData());
                spectral_interface->transmit(data_1);

                Packet data_2(flagged_address_t(standard_address_t(space_resources::SYSTEM_ID, space_resources::controller::SUBSYSTEM_ID, space_resources::controller::sensing::DEVICE_ID, space_resources::controller::sensing::group::SPECTRAL_SENSOR, space_resources::controller::sensing::spectral_sensor_parameter::DATA2)), parameters::space_resources::controller::sensing::spectrum_2_t {
                        .vis_1 = data[4];
                        .flicker_detection_1 = data[5];
                        .dark_blue_425nm = data[6];
                        .light_blue_475nm = data[7]; }.serializeData());
                spectral_interface->transmit(data_2);

                Packet data_3(flagged_address_t(standard_address_t(space_resources::SYSTEM_ID, space_resources::controller::SUBSYSTEM_ID, space_resources::controller::sensing::DEVICE_ID, space_resources::controller::sensing::group::SPECTRAL_SENSOR, space_resources::controller::sensing::spectral_sensor_parameter::DATA3)), parameters::space_resources::controller::sensing::spectrum_3_t {
                        .blue_515nm = data[8];
                        .brown_640nm = data[9];
                        .vis_2 = data[10];
                        .flicker_detection_2 = data[11]; }.serializeData());
                spectral_interface->transmit(data_3);

                Packet data_4(flagged_address_t(standard_address_t(space_resources::SYSTEM_ID, space_resources::controller::SUBSYSTEM_ID, space_resources::controller::sensing::DEVICE_ID, space_resources::controller::sensing::group::SPECTRAL_SENSOR, space_resources::controller::sensing::spectral_sensor_parameter::DATA4)), parameters::space_resources::controller::sensing::spectrum_4_t {
                        .purple_405nm = data[12];
                        .red_690nm = data[13];
                        .dark_red_745nm = data[14];
                        .green_550nm = data[15]; }.serializeData());
                spectral_interface->transmit(data_4);

                Packet data_5(flagged_address_t(standard_address_t(space_resources::SYSTEM_ID, space_resources::controller::SUBSYSTEM_ID, space_resources::controller::sensing::DEVICE_ID, space_resources::controller::sensing::group::SPECTRAL_SENSOR, space_resources::controller::sensing::spectral_sensor_parameter::DATA5)), parameters::space_resources::controller::sensing::spectrum_5_t {
                        .vis_3 = data[16];
                        .flicker_detection_3 = data[17]; }.serializeData());
                spectral_interface->transmit(data_5);
            }});
    packetManager->setCallback(addressing::filter_t{.address = flagged_address_t(standard_address_t(space_resources::SYSTEM_ID, space_resources::controller::SUBSYSTEM_ID, space_resources::controller::sensing::DEVICE_ID, space_resources::controller::sensing::group::MAGNETOMETER, space_resources::controller::sensing::magnetometer_sensor_parameter::START))}, callback_config_t{.datacallback = [&](const Packet)
                                                                                                                                                                                                                                                                                                                                                                                        {
                                                                                                                                                                                                                                                                                                                                                                                            start_reading_magnetometer();
                                                                                                                                                                                                                                                                                                                                                                                            uint16_t* data = get_magnetometer_readings();
                                                                                                                                                                                                                                                                                                                                                                                            // uint16_t data[18] = get_reading();
                                                                                                                                                                                                                                                                                                                                                                                            Packet xyz(flagged_address_t(standard_address_t(space_resources::SYSTEM_ID, space_resources::controller::SUBSYSTEM_ID, space_resources::controller::sensing::DEVICE_ID, space_resources::controller::sensing::group::MAGNETOMETER, space_resources::controller::sensing::magnetometer_sensor_parameter::XYZ)), parameters::space_resources::controller::sensing::magnet_t {
                        .x = PLACEHOLDER;
                        .y = PLACEHOLDER;
                        .z = PLACEHOLDER; }.serializeData());
                                                                                                                                                                                                                                                                                                                                                                                            spectral_interface->transmit(xyz);
                                                                                                                                                                                                                                                                                                                                                                                        }})
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

void readSpectral()
{
    Wire.beginTransmission(spectral_address);
    Wire.write(spectral_data_N_L_register)
        Wire.endTransmission();

    int bytesToRead = 36;
    Wire.requestFrom(spectral_address, bytesToRead);

    if (Wire.available == bytesToRead)
    {
        for (int i = 0; i < 18; i++)
        {
            uint16_t low = Wire.read();
            uint16_t high = Wire.read();
            uint16_t value = (high << 8) | low;

            Serial.print("Channel");
            Serial.print(i);
            Serial.print(": ");
            Serial.print(value);
            Serial.print("\t");
        }
        Serial.println();
    }
    else
    {
        Serial.println("Error reading spectral sensor")
    }
}

void readMagnetometer()
{
    // writeToMagnetometer(magnetometer_address, 0b00111111)
    writeToMagnetometer(magnetometer_address, 0b00111111);
    writeToMagnetometer(magnetometer_address, (magnetometer_enable_data | magnetometer_enable_xyz));

    Wire.requestFrom(magnetometer_address, 9);

    if (Wire.available() == 9)
    {
        byte status = Wire.read();
        // int16_t temp = (Wire.read() << 8) | Wire.read();
        int16_t x = (Wire.read() << 8) | Wire.read();
        int16_t y = (Wire.read() << 8) | Wire.read();
        int16_t z = (Wire.read() << 8) | Wire.read();
        Serial.print("X: ");
        Serial.print(x);
        Serial.print(" | Y: ");
        Serial.print(y);
        Serial.print(" | Z: ");
        Serial.println(z);
    }
    else
    {
        Serial.println("Error reading magnetometer.");
    }
}

void loop()
{
    packet_manager->handle();

    readSpectral();
    readMagnetometer();
}