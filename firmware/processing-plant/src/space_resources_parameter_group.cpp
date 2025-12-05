#include "space_resources_parameter_groups.hpp"

CentrifugeParameterGroup::CentrifugeParameterGroup(const hi_can::addressing::standard_address_t deviceAddress, const hi_can::addressing::space_resources::controller::centrifuge::group motorGroup, const gpio_num_t motorPin)
    : _deviceAddress(deviceAddress),
      _motorGroup(motorGroup),
      _motorPin(motorPin)
{
    _callbacks.emplace_back(
        filter_t{
            .address = standard_address_t(deviceAddress, static_cast<uint8_t>(motorGroup), static_cast<uint8_t>(hi_can::addressing::space_resources::controller::centrifuge::motor_parameter::SPEED))},
        PacketManager::callback_config_t{
            .dataCallback = [this](const Packet& packet)
            {
                _motorSpeed.deserialiseData(packet.getData());
            },
        });
}
MagnetometerParameterGroup::MagnetometerParameterGroup(const hi_can::addressing::standard_address_t deviceAddress, const hi_can::addressing::space_resources::controller::sensing::group sensorGroup, const gpio_num_t sensorPin)
    : _deviceAddress(deviceAddress),
      _sensorGroup(sensorGroup),
      _sensorPin(sensorPin)
{
    _callbacks.emplace_back(
        filter_t{
            .address = standard_address_t(deviceAddress, static_cast<uint8_t>(sensorGroup), static_cast<uint8_t>(hi_can::addressing::space_resources::controller::sensing::sensor_parameter::START))},
        PacketManager::callback_config_t{
            .dataCallback = [this](const Packet& packet)
            {
                // Do something
            },
        });
}
SpectralSensorParameterGroup::SpectralSensorParameterGroup(const hi_can::addressing::standard_address_t deviceAddress, const hi_can::addressing::space_resources::controller::sensing::group sensorGroup, const gpio_num_t sensorPin)
    : _deviceAddress(deviceAddress),
      _sensorGroup(sensorGroup),
      _sensorPin(sensorPin)
{
    _callbacks.emplace_back(
        filter_t{
            .address = standard_address_t(deviceAddress, static_cast<uint8_t>(sensorGroup), static_cast<uint8_t>(hi_can::addressing::space_resources::controller::sensing::sensor_parameter::START))},
        PacketManager::callback_config_t{
            .dataCallback = [this](const Packet& packet)
            {
                // Do something
            },
        });
}
IlmeniteActuatorParameterGroup::IlmeniteActuatorParameterGroup(const hi_can::addressing::standard_address_t deviceAddress, const hi_can::addressing::space_resources::controller::sensing::group sensorGroup, const gpio_num_t sensorPin)
    : _deviceAddress(deviceAddress),
      _sensorGroup(sensorGroup),
      _sensorPin(sensorPin)
{
    _callbacks.emplace_back(
        filter_t{
            .address = standard_address_t(deviceAddress, static_cast<uint8_t>(sensorGroup), static_cast<uint8_t>(hi_can::addressing::space_resources::controller::sensing::actuator_parameter::START))},
        PacketManager::callback_config_t{
            .dataCallback = [this](const Packet& packet)
            {
                // Do something
            },
        });
}