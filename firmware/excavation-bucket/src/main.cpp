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
static constexpr uint32_t PWM_FREQ = 1500;  // Hz

static constexpr uint32_t PWM_DEADBAND = 2;                              // 2 PWM steps of enforced deadband to reset cycle-by-cycle current chopping
static constexpr uint32_t PWM_MAX = (1 << PWM_BITS) - 1 - PWM_DEADBAND;  // 4095

class MotorDriver
{
public:
    enum class direction
    {
        FORWARD,
        STOPPED,
        BACKWARD
    };
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

        direction currentDir = direction::STOPPED;
        if (speed > 0)
            currentDir = direction::FORWARD;
        else if (speed < 0)
            currentDir = direction::BACKWARD;
        bool dirChanged = (currentDir != _prevDirection);
        _prevDirection = currentDir;

        if (dirChanged)
        {
            if (currentDir == direction::FORWARD)
            {
                pinMode(_pins.second, OUTPUT);
                digitalWrite(_pins.second, LOW);
                analogWrite(_pins.first, 1);
                analogWriteResolution(_pins.first, PWM_BITS);
                analogWriteFrequency(_pins.first, PWM_FREQ);
            }
            else if (currentDir == direction::BACKWARD)
            {
                pinMode(_pins.first, OUTPUT);
                digitalWrite(_pins.first, LOW);
                analogWrite(_pins.second, 1);
                analogWriteResolution(_pins.second, PWM_BITS);
                analogWriteFrequency(_pins.second, PWM_FREQ);
            }
            else
            {
                pinMode(_pins.first, OUTPUT);
                pinMode(_pins.second, OUTPUT);
                digitalWrite(_pins.first, LOW);
                digitalWrite(_pins.second, LOW);
            }
        }

        if (speed > 0)
            analogWrite(_pins.first, speed);
        else if (speed < 0)
            analogWrite(_pins.second, -speed);
    }

private:
    direction _prevDirection = direction::STOPPED;

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
            .sample_rate_hz = 1000 * 1000,
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
        sdm_channel_set_pulse_density(_currentLimitChannel, -128);
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

    float getCurrentLimit() const
    {
        return _currentLimit;
    }
    void setCurrentLimit(float limit)
    {
        if (limit > MAX_CURRENT)
            limit = MAX_CURRENT;
        _currentLimit = limit;
        float voltage = currentToVoltage(limit);
        const int8_t pwmValue = static_cast<int8_t>(
            std::clamp((voltage * 255 / MAX_VOLTAGE) - 128,
                       static_cast<float>(std::numeric_limits<int8_t>::min()),
                       static_cast<float>(std::numeric_limits<int8_t>::max())));
        sdm_channel_set_pulse_density(_currentLimitChannel, pwmValue);
        printf(std::format("Set current limit to {:.02f} {:.02f} {}\n", limit, voltage, pwmValue).c_str());
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
    float _currentLimit = 0.0f;
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
using namespace hi_can::addressing;

std::optional<PacketManager> packetManager;

void handleMotorSpeedData(const Packet& packet);
void handleMotorCurrentData(const Packet& packet);
void setMotorSpeed(const excavation::bucket::controller::group& group,
                   const int16_t& speed);
void setMotorCurrent(const excavation::bucket::controller::group& group,
                     const uint16_t& current);
void registerMotorBank(const excavation::bucket::controller::group& group,
                       const uint8_t& speedParam);

std::optional<MotorBank> motorBank1;
std::optional<MotorBank> motorBank2;
std::optional<MotorBank> motorBank3;

constexpr standard_address_t DEVICE_ADDRESS{
    excavation::SYSTEM_ID,
    excavation::bucket::SUBSYSTEM_ID,
    excavation::bucket::controller::DEVICE_ID,
};

void setup()
{
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

    pinMode(MAGNET_PIN, OUTPUT);
    digitalWrite(MAGNET_PIN, LOW);

    auto& interface = TwaiInterface::getInstance(std::make_pair(bsp::CAN_TX_PIN, bsp::CAN_RX_PIN), 0,
                                                 filter_t{
                                                     .address = static_cast<flagged_address_t>(DEVICE_ADDRESS),
                                                     .mask = DEVICE_MASK,
                                                 });
    packetManager.emplace(interface);

    using namespace excavation::bucket::controller;
    const std::vector<group> actuatorGroups = {
        group::LIFT_BOTH,
        group::LIFT_LEFT,
        group::LIFT_RIGHT,
        group::TILT_BOTH,
        group::TILT_LEFT,
        group::TILT_RIGHT,
        group::JAWS_BOTH,
        group::JAWS_LEFT,
        group::JAWS_RIGHT,
    };

    for (const auto& group : actuatorGroups)
        registerMotorBank(group, static_cast<uint8_t>(actuator_parameter::SPEED));
    registerMotorBank(group::MAGNET,
                      static_cast<uint8_t>(magnet_parameter::ROTATE_SPEED));

    using namespace parameters::excavation::bucket::controller;
    packetManager->setTransmissionConfig(
        static_cast<flagged_address_t>(
            standard_address_t{DEVICE_ADDRESS,
                               static_cast<uint8_t>(group::BANK_1),
                               static_cast<uint8_t>(bank_parameter::CURRENT_LIMIT)}),
        {
            .generator = [=]()
            {
                return current_t{static_cast<uint16_t>(std::clamp(motorBank1->getAverageCurrent() * 1000,
                                                                  static_cast<float>(std::numeric_limits<uint16_t>::min()),
                                                                  static_cast<float>(std::numeric_limits<uint16_t>::max())))}
                    .serializeData();
            },
            .interval = 100ms,
            .shouldTransmitImmediately = true,
        });
    packetManager->setCallback(
        filter_t{static_cast<flagged_address_t>(
            standard_address_t{DEVICE_ADDRESS,
                               static_cast<uint8_t>(group::BANK_1),
                               static_cast<uint8_t>(bank_parameter::CURRENT_LIMIT)})},
        {
            .dataCallback = handleMotorCurrentData,
        });
    packetManager->setTransmissionConfig(
        static_cast<flagged_address_t>(
            standard_address_t{DEVICE_ADDRESS,
                               static_cast<uint8_t>(group::BANK_2),
                               static_cast<uint8_t>(bank_parameter::STATUS)}),
        {
            .generator = [=]()
            {
                return current_t{static_cast<uint16_t>(std::clamp(motorBank2->getAverageCurrent() * 1000,
                                                                  static_cast<float>(std::numeric_limits<uint16_t>::min()),
                                                                  static_cast<float>(std::numeric_limits<uint16_t>::max())))}
                    .serializeData();
            },
            .interval = 100ms,
            .shouldTransmitImmediately = true,
        });
    packetManager->setCallback(
        filter_t{static_cast<flagged_address_t>(
            standard_address_t{DEVICE_ADDRESS,
                               static_cast<uint8_t>(group::BANK_2),
                               static_cast<uint8_t>(bank_parameter::CURRENT_LIMIT)})},
        {
            .dataCallback = handleMotorCurrentData,
        });
    packetManager->setTransmissionConfig(
        static_cast<flagged_address_t>(
            standard_address_t{DEVICE_ADDRESS,
                               static_cast<uint8_t>(group::BANK_3),
                               static_cast<uint8_t>(bank_parameter::STATUS)}),
        {
            .generator = [=]()
            {
                return current_t{static_cast<uint16_t>(std::clamp(motorBank3->getAverageCurrent() * 1000,
                                                                  static_cast<float>(std::numeric_limits<uint16_t>::min()),
                                                                  static_cast<float>(std::numeric_limits<uint16_t>::max())))}
                    .serializeData();
            },
            .interval = 100ms,
            .shouldTransmitImmediately = true,
        });
    packetManager->setCallback(
        filter_t{static_cast<flagged_address_t>(
            standard_address_t{DEVICE_ADDRESS,
                               static_cast<uint8_t>(group::BANK_3),
                               static_cast<uint8_t>(bank_parameter::CURRENT_LIMIT)})},
        {
            .dataCallback = handleMotorCurrentData,
        });
}

void loop()
{
    packetManager->handle();
    delay(1);
}

void handleMotorSpeedData(const Packet& packet)
{
    using namespace excavation::bucket::controller;
    using namespace hi_can::parameters::excavation::bucket::controller;
    try
    {
        standard_address_t address{packet.getAddress().address};
        setMotorSpeed(
            static_cast<group>(standard_address_t(packet.getAddress().address).group),
            speed_t{packet.getData()}.value);
    }
    catch (const std::exception& e)
    {
        printf(std::format("Failed to parse speed packet: {}\n", e.what()).c_str());
    }
}
void handleMotorCurrentData(const Packet& packet)
{
    using namespace excavation::bucket::controller;
    using namespace hi_can::parameters::excavation::bucket::controller;
    try
    {
        standard_address_t address{packet.getAddress().address};
        setMotorCurrent(static_cast<group>(
                            standard_address_t(packet.getAddress().address).group),
                        current_t{packet.getData()}.value);
    }
    catch (const std::exception& e)
    {
        printf(std::format("Failed to parse current packet: {}\n", e.what()).c_str());
    }
}

void setMotorSpeed(const excavation::bucket::controller::group& group, const int16_t& speed)
{
    using namespace excavation::bucket::controller;
    // printf(std::format("Setting motor (group) {:#x} to speed {}\n",
    //                    static_cast<uint8_t>(group), speed)
    //            .c_str());
    switch (group)
    {
    case group::LIFT_BOTH:
        setMotorSpeed(group::LIFT_LEFT, speed);
        setMotorSpeed(group::LIFT_RIGHT, speed);
        break;
    case group::LIFT_LEFT:
        motorBank1->setSpeedA(speed);
        break;
    case group::LIFT_RIGHT:
        motorBank1->setSpeedB(speed);
        break;
    case group::TILT_BOTH:
        motorBank3->setSpeedA(speed);
        break;
    case group::JAWS_BOTH:
        setMotorSpeed(group::JAWS_LEFT, speed);
        setMotorSpeed(group::JAWS_RIGHT, speed);
        break;
    case group::JAWS_LEFT:
        motorBank2->setSpeedA(speed);
        break;
    case group::JAWS_RIGHT:
        motorBank2->setSpeedB(speed);
        break;
    case group::MAGNET:
        motorBank3->setSpeedB(speed);
        break;
    default:
        break;
    }
}
void setMotorCurrent(const excavation::bucket::controller::group& group,
                     const uint16_t& current)
{
    using namespace excavation::bucket::controller;
    printf(std::format("Setting {:#x} current to {}mA\n", static_cast<uint8_t>(group), current).c_str());
    switch (group)
    {
    case group::BANK_1:
        motorBank1->setCurrentLimit(current / 1000.0f);
        break;
    case group::BANK_2:
        motorBank2->setCurrentLimit(current / 1000.0f);
        break;
    case group::BANK_3:
        motorBank3->setCurrentLimit(current / 1000.0f);
        break;
    default:
        break;
    }
}

void registerMotorBank(const excavation::bucket::controller::group& group,
                       const uint8_t& speedParam)
{
    const standard_address_t speedAddress{DEVICE_ADDRESS,
                                          static_cast<uint8_t>(group),
                                          static_cast<uint8_t>(speedParam)};
    packetManager->setCallback(filter_t{
                                   static_cast<flagged_address_t>(speedAddress),
                               },
                               {
                                   .dataCallback = handleMotorSpeedData,
                                   .timeoutCallback = std::bind(setMotorSpeed, group, (int16_t)0),
                                   .timeout = 200ms,
                               });
}