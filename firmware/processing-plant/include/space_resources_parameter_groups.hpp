#include <board_support.hpp>
#include <hi_can.hpp>

class CentrifugeParameterGroup : public hi_can::parameters::ParameterGroup
{
public:
    CentrifugeParameterGroup(const hi_can::addressing::standard_address_t deviceAddress, const hi_can::addressing::space_resources::controller::centrifuge::group motorGroup, const gpio_num_t motorPin);

private:
    const hi_can::addressing::standard_address_t _deviceAddress;
    const hi_can::addressing::space_resources::controller::centrifuge::group _motorGroup;
    const gpio_num_t _motorPin;
    hi_can::parameters::space_resources::controller::centrifuge::speed_t _motorSpeed = 0;
};
class MagnetometerParameterGroup : public hi_can::parameters::ParameterGroup
{
public:
    MagnetometerParameterGroup(const hi_can::addressing::standard_address_t deviceAddress, const hi_can::addressing::space_resources::controller::sensing::group sensorGroup, const gpio_num_t sensorPin);

private:
    const hi_can::addressing::standard_address_t _deviceAddress;
    const hi_can::addressing::space_resources::controller::sensing::group _sensorGroup;
    const gpio_num_t _sensorPin;
};
class SpectralSensorParameterGroup : public hi_can::parameters::ParameterGroup
{
public:
    SpectralSensorParameterGroup(const hi_can::addressing::standard_address_t deviceAddress, const hi_can::addressing::space_resources::controller::sensing::group sensorGroup, const gpio_num_t sensorPin);

private:
    const hi_can::addressing::standard_address_t _deviceAddress;
    const hi_can::addressing::space_resources::controller::sensing::group _sensorGroup;
    const gpio_num_t _sensorPin;
};
class IlmeniteActuatorParameterGroup : public hi_can::parameters::ParameterGroup
{
public:
    IlmeniteActuatorParameterGroup(const hi_can::addressing::standard_address_t deviceAddress, const hi_can::addressing::space_resources::controller::sensing::group actuatorGroup, const gpio_num_t actuatorPin);

private:
    const hi_can::addressing::standard_address_t _deviceAddress;
    const hi_can::addressing::space_resources::controller::sensing::group _actuatorGroup;
    const gpio_num_t _actuatorPin;
};