#include <Arduino.h>
#include <driver/sdm.h>

#include <board_support.hpp>
#include <chrono>
#include <hi_can_twai.hpp>
#include <optional>
#include <thread>

using namespace bsp;

static constexpr pin_pair_t DRIVER_1_PINS{GPIO_NUM_15, GPIO_NUM_16};
static constexpr pin_pair_t DRIVER_2_PINS{GPIO_NUM_42, GPIO_NUM_41};
static constexpr pin_pair_t DRIVER_3_PINS{GPIO_NUM_38, GPIO_NUM_37};
static constexpr pin_pair_t DRIVER_4_PINS{GPIO_NUM_45, GPIO_NUM_48};
static constexpr pin_pair_t DRIVER_5_PINS{GPIO_NUM_47, GPIO_NUM_21};
static constexpr pin_pair_t DRIVER_6_PINS{GPIO_NUM_14, GPIO_NUM_13};
static constexpr gpio_num_t MAGNET_PIN = bsp::A9;

static constexpr gpio_num_t BANK_1_CURRENT_LIMIT = GPIO_NUM_40;
static constexpr gpio_num_t BANK_2_CURRENT_LIMIT = bsp::A10;
static constexpr gpio_num_t BANK_3_CURRENT_LIMIT = GPIO_NUM_12;

static constexpr gpio_num_t BANK_1_CURRENT_SENSE = bsp::A1;
static constexpr gpio_num_t BANK_2_CURRENT_SENSE = bsp::A3;
static constexpr gpio_num_t BANK_3_CURRENT_SENSE = bsp::A5;
static constexpr gpio_num_t MAGNET_CURRENT_SENSE = bsp::A7;

static constexpr gpio_num_t BANK_1_FAULT = bsp::A2;
static constexpr gpio_num_t BANK_2_FAULT = bsp::A4;
static constexpr gpio_num_t BANK_3_FAULT = bsp::A6;

static constexpr gpio_num_t SLEEP = GPIO_NUM_39;

static constexpr uint8_t PWM_BITS = 12;
static constexpr uint32_t PWM_FREQ = 1000;  // Hz

static constexpr uint32_t PWM_MAX = (1 << PWM_BITS) - 1;  // 4095

class MotorDriver
{
public:
    MotorDriver(const pin_pair_t& pins)
        : _pins(pins)
    {
        pinMode(pins.first, OUTPUT);
        pinMode(pins.second, OUTPUT);
    }

    virtual ~MotorDriver()
    {
        digitalWrite(_pins.first, LOW);
        digitalWrite(_pins.second, LOW);
        pinMode(_pins.first, INPUT);
        pinMode(_pins.second, INPUT);
    }

    void setSpeed(int16_t speed)
    {
        speed = map(speed, std::numeric_limits<int16_t>::min(),
                    std::numeric_limits<int16_t>::max(),
                    -PWM_MAX,
                    PWM_MAX);

        if (speed > 0)
        {
            digitalWrite(_pins.second, LOW);
            analogWrite(_pins.first, speed);
            analogWriteResolution(_pins.first, PWM_BITS);
            analogWriteFrequency(_pins.first, PWM_FREQ);
        }
        else if (speed < 0)
        {
            digitalWrite(_pins.first, LOW);
            analogWrite(_pins.second, -speed);
            analogWriteResolution(_pins.second, PWM_BITS);
            analogWriteFrequency(_pins.second, PWM_FREQ);
        }
        else
        {
            digitalWrite(_pins.first, LOW);
            digitalWrite(_pins.second, LOW);
        }
    }

private:
    pin_pair_t _pins;
};

class MotorBank
{
public:
    static constexpr uint16_t CURRENT_SENSE_RESISTOR = 1000;        // ohms
    static constexpr float CURRENT_SENSE_PROPORTIONALITY = 450e-6;  // amps per amp

    // static constexpr float currentToVoltage(const float& current);
    // static constexpr float voltageToCurrent(const float& voltage);
    static constexpr float currentToVoltage(const float& current)
    {
        return current * (CURRENT_SENSE_RESISTOR * CURRENT_SENSE_PROPORTIONALITY);
    }
    static constexpr float voltageToCurrent(const float& voltage)
    {
        return voltage / (CURRENT_SENSE_RESISTOR * CURRENT_SENSE_PROPORTIONALITY);
    }

    static constexpr float MAX_VOLTAGE = 3.3f;  // volts
    static constexpr float MAX_CURRENT = 6.0f;  // amps

    MotorBank(const pin_pair_t& driverAPins,
              const pin_pair_t& driverBPins,
              const gpio_num_t& currentLimitPin,
              const gpio_num_t& currentSensePin,
              const gpio_num_t& faultPin)
        : _driverA(driverAPins),
          _driverB(driverBPins),
          _currentLimitPin(currentLimitPin),
          _currentSensePin(currentSensePin),
          _faultPin(faultPin)
    {
        pinMode(_currentLimitPin, OUTPUT);
        pinMode(_currentSensePin, INPUT);
        pinMode(_faultPin, INPUT_PULLUP);

        setSpeed(0, 0);

        sdm_config_t config = {
            .gpio_num = _currentLimitPin,
            .clk_src = SDM_CLK_SRC_DEFAULT,
            .sample_rate_hz = 1 * 1000 * 1000,
        };
        esp_err_t err = sdm_new_channel(&config, &_currentLimitChannel);
        if (err != ESP_OK)
        {
            throw std::runtime_error(std::format("Failed to create SDM channel: {}", esp_err_to_name(err)));
        }

        sdm_channel_enable(_currentLimitChannel);
        if (err != ESP_OK)
        {
            throw std::runtime_error(std::format("Failed to enable SDM channel: {}", esp_err_to_name(err)));
        }
        setCurrentLimit(MAX_CURRENT);
    }

    // delete copy/move semantics
    MotorBank(const MotorBank&) = delete;
    MotorBank(MotorBank&&) = delete;
    MotorBank& operator=(const MotorBank&) = delete;
    MotorBank& operator=(MotorBank&&) = delete;

    virtual ~MotorBank()
    {
        sdm_channel_set_duty(_currentLimitChannel, -128);
        sdm_channel_disable(_currentLimitChannel);
        sdm_del_channel(_currentLimitChannel);
        pinMode(_currentLimitPin, INPUT);
        pinMode(_currentSensePin, INPUT);
        pinMode(_faultPin, INPUT_PULLUP);
    }

    void setSpeed(int16_t speedA, int16_t speedB)
    {
        _driverA.setSpeed(speedA);
        _driverB.setSpeed(speedB);
    }
    void setSpeedA(int16_t speed)
    {
        _driverA.setSpeed(speed);
    }
    void setSpeedB(int16_t speed)
    {
        _driverB.setSpeed(speed);
    }

    void setCurrentLimit(float limit)
    {
        if (limit > MAX_CURRENT)
            limit = MAX_CURRENT;
        if (limit < 0)
            limit = 0;
        float voltage = currentToVoltage(limit);
        if (voltage > 3.3f)
            voltage = 3.3f;
        int8_t pwmValue = static_cast<int8_t>(map(voltage, 0, 3.3f, std::numeric_limits<int8_t>::min(), std::numeric_limits<int8_t>::max()));
        sdm_channel_set_duty(_currentLimitChannel, pwmValue);
    }
    float getAverageCurrent()
    {
        return voltageToCurrent(analogReadMilliVolts(_currentSensePin));
    }

    bool isInFault()
    {
        return digitalRead(_faultPin) == LOW;
    }

private:
    sdm_channel_handle_t _currentLimitChannel;

    MotorDriver _driverA;
    MotorDriver _driverB;

    const gpio_num_t _currentLimitPin;
    const gpio_num_t _currentSensePin;
    const gpio_num_t _faultPin;
};

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace hi_can;

std::optional<PacketManager> packetManager;

void handleMotorSpeedData(const Packet& packet);
void setMotorSpeed(const uint8_t& group, const int16_t& speed);

std::optional<MotorBank> motorBank1;
std::optional<MotorBank> motorBank2;
std::optional<MotorBank> motorBank3;

constexpr addressing::standard_address_t DEVICE_ADDRESS{
    addressing::excavation::SYSTEM_ID,
    addressing::excavation::bucket::SUBSYSTEM_ID,
    addressing::excavation::bucket::controller::DEVICE_ID,
};

void setup()
{
    auto& interface = TwaiInterface::getInstance();
    packetManager.emplace(interface);
    using namespace addressing;
    using namespace excavation::bucket::controller;
    std::vector<actuators::group> groups = {
        actuators::group::LIFT_MAIN,
        actuators::group::TILT_MAIN,
        actuators::group::JAWS_MAIN,
    };
    for (const auto& group : groups)
    {
        standard_address_t address{DEVICE_ADDRESS,
                                   static_cast<uint8_t>(group),
                                   static_cast<uint8_t>(actuators::parameter::SPEED)};
        packetManager->setCallback(filter_t{
                                       static_cast<flagged_address_t>(address),
                                   },
                                   {
                                       .dataCallback = handleMotorSpeedData,
                                       .timeoutCallback = std::bind(setMotorSpeed, static_cast<uint8_t>(group), (int16_t)0),
                                       .timeout = 200ms,
                                   });
    }
    // reset drivers
    pinMode(SLEEP, OUTPUT);
    digitalWrite(SLEEP, LOW);
    delay(100);
    digitalWrite(SLEEP, HIGH);
    motorBank1.emplace(DRIVER_1_PINS,
                       DRIVER_2_PINS,
                       BANK_1_CURRENT_LIMIT,
                       BANK_1_CURRENT_SENSE,
                       BANK_1_FAULT);
    motorBank2.emplace(DRIVER_3_PINS,
                       DRIVER_4_PINS,
                       BANK_2_CURRENT_LIMIT,
                       BANK_2_CURRENT_SENSE,
                       BANK_2_FAULT);
    motorBank3.emplace(DRIVER_5_PINS,
                       DRIVER_6_PINS,
                       BANK_3_CURRENT_LIMIT,
                       BANK_3_CURRENT_SENSE,
                       BANK_3_FAULT);
}

void loop()
{
    packetManager->handle(true);
    delay(1);
}

void handleMotorSpeedData(const Packet& packet)
{
    using namespace addressing;
    using namespace excavation::bucket::controller;
    using namespace hi_can::parameters::excavation::bucket::controller;
    try
    {
        standard_address_t address{packet.getAddress().address};
        setMotorSpeed(static_cast<uint8_t>(
                          standard_address_t(packet.getAddress().address).group),
                      speed_t{packet.getData()}.value);
    }
    catch (const std::exception& e)
    {
        printf(std::format("Failed to parse speed packet: {}\n", e.what()).c_str());
    }
}

void setMotorSpeed(const uint8_t& group, const int16_t& speed)
{
    using namespace hi_can::addressing;
    using namespace excavation::bucket::controller;
    switch (group)
    {
    case static_cast<uint8_t>(actuators::group::LIFT_MAIN):
        setMotorSpeed(static_cast<uint8_t>(actuators::group::LIFT_LEFT), speed);
        setMotorSpeed(static_cast<uint8_t>(actuators::group::LIFT_RIGHT), speed);
        break;
    case static_cast<uint8_t>(actuators::group::LIFT_LEFT):
        motorBank1->setSpeedA(speed);
        break;
    case static_cast<uint8_t>(actuators::group::LIFT_RIGHT):
        motorBank2->setSpeedA(speed);
        break;
    case static_cast<uint8_t>(actuators::group::TILT_MAIN):
        motorBank3->setSpeedB(speed);
        break;
    case static_cast<uint8_t>(actuators::group::JAWS_MAIN):
        setMotorSpeed(static_cast<uint8_t>(actuators::group::JAWS_LEFT), speed);
        setMotorSpeed(static_cast<uint8_t>(actuators::group::JAWS_RIGHT), speed);
        break;
    case static_cast<uint8_t>(actuators::group::JAWS_LEFT):
        motorBank2->setSpeedB(speed);
        break;
    case static_cast<uint8_t>(actuators::group::JAWS_RIGHT):
        motorBank3->setSpeedA(speed);
        break;
    case static_cast<uint8_t>(magnet::GROUP_ID):
        motorBank1->setSpeedB(speed);
        break;
    default:
        break;
    }
}
