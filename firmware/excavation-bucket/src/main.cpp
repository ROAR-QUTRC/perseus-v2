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

    void set_speed(int16_t speed)
    {
        speed = map(speed, std::numeric_limits<int16_t>::min(),
                    std::numeric_limits<int16_t>::max(),
                    -PWM_MAX,
                    PWM_MAX);

        direction current_dir = direction::STOPPED;
        if (speed > 0)
            current_dir = direction::FORWARD;
        else if (speed < 0)
            current_dir = direction::BACKWARD;
        bool dir_changed = (current_dir != _prev_direction);
        _prev_direction = current_dir;

        if (dir_changed)
        {
            if (current_dir == direction::FORWARD)
            {
                pinMode(_pins.second, OUTPUT);
                digitalWrite(_pins.second, LOW);
                analogWrite(_pins.first, 1);
                analogWriteResolution(_pins.first, PWM_BITS);
                analogWriteFrequency(_pins.first, PWM_FREQ);
            }
            else if (current_dir == direction::BACKWARD)
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
    direction _prev_direction = direction::STOPPED;

    pin_pair_t _pins;
};

class MotorBank
{
public:
    static constexpr uint16_t CURRENT_SENSE_RESISTOR = 1000;        // ohms
    static constexpr float CURRENT_SENSE_PROPORTIONALITY = 450e-6;  // amps per amp

    // static constexpr float current_to_voltage(const float& current);
    // static constexpr float voltage_to_current(const float& voltage);
    static constexpr float current_to_voltage(const float& current)
    {
        return current * (CURRENT_SENSE_RESISTOR * CURRENT_SENSE_PROPORTIONALITY);
    }
    static constexpr float voltage_to_current(const float& voltage)
    {
        return voltage / (CURRENT_SENSE_RESISTOR * CURRENT_SENSE_PROPORTIONALITY);
    }

    static constexpr float MAX_VOLTAGE = 3.3f;  // volts
    static constexpr float MAX_CURRENT = 6.0f;  // amps

    MotorBank(const pin_pair_t& driver_A_pins,
              const pin_pair_t& driver_B_pins,
              const gpio_num_t& current_limit_pin,
              const gpio_num_t& current_sense_pin,
              const gpio_num_t& fault_pin)
        : _driver_A(driver_A_pins),
          _driver_B(driver_B_pins),
          _current_limit_pin(current_limit_pin),
          _current_sense_pin(current_sense_pin),
          _fault_pin(fault_pin)
    {
        pinMode(_current_limit_pin, OUTPUT);
        pinMode(_current_sense_pin, INPUT);
        pinMode(_fault_pin, INPUT_PULLUP);

        set_speed(0, 0);

        sdm_config_t config = {
            .gpio_num = _current_limit_pin,
            .clk_src = SDM_CLK_SRC_DEFAULT,
            .sample_rate_hz = 1000 * 1000,
        };
        esp_err_t err = sdm_new_channel(&config, &_current_limit_channel);
        if (err != ESP_OK)
        {
            throw std::runtime_error(std::format("Failed to create SDM channel: {}", esp_err_to_name(err)));
        }

        sdm_channel_enable(_current_limit_channel);
        if (err != ESP_OK)
        {
            throw std::runtime_error(std::format("Failed to enable SDM channel: {}", esp_err_to_name(err)));
        }
        set_current_limit(MAX_CURRENT);
    }

    // delete copy/move semantics
    MotorBank(const MotorBank&) = delete;
    MotorBank(MotorBank&&) = delete;
    MotorBank& operator=(const MotorBank&) = delete;
    MotorBank& operator=(MotorBank&&) = delete;

    virtual ~MotorBank()
    {
        sdm_channel_set_pulse_density(_current_limit_channel, -128);
        sdm_channel_disable(_current_limit_channel);
        sdm_del_channel(_current_limit_channel);
        pinMode(_current_limit_pin, INPUT);
        pinMode(_current_sense_pin, INPUT);
        pinMode(_fault_pin, INPUT_PULLUP);
    }

    void set_speed(int16_t speed_a, int16_t speed_b)
    {
        _driver_A.set_speed(speed_a);
        _driver_B.set_speed(speed_b);
    }
    void set_speed_a(int16_t speed)
    {
        _driver_A.set_speed(speed);
    }
    void set_speed_b(int16_t speed)
    {
        _driver_B.set_speed(speed);
    }

    float get_current_limit() const
    {
        return _current_limit;
    }
    void set_current_limit(float limit)
    {
        if (limit > MAX_CURRENT)
            limit = MAX_CURRENT;
        _current_limit = limit;
        float voltage = current_to_voltage(limit);
        const int8_t pwm_value = static_cast<int8_t>(
            std::clamp((voltage * 255 / MAX_VOLTAGE) - 128,
                       static_cast<float>(std::numeric_limits<int8_t>::min()),
                       static_cast<float>(std::numeric_limits<int8_t>::max())));
        sdm_channel_set_pulse_density(_current_limit_channel, pwm_value);
        printf(std::format("Set current limit to {:.02f} {:.02f} {}\n", limit, voltage, pwm_value).c_str());
    }
    float get_average_current()
    {
        return voltage_to_current(analogReadMilliVolts(_current_sense_pin) / 1000.0f);
    }

    bool is_in_fault()
    {
        return digitalRead(_fault_pin) == LOW;
    }

private:
    float _current_limit = 0.0f;
    sdm_channel_handle_t _current_limit_channel;

    MotorDriver _driver_A;
    MotorDriver _driver_B;

    const gpio_num_t _current_limit_pin;
    const gpio_num_t _current_sense_pin;
    const gpio_num_t _fault_pin;
};

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace hi_can;
using namespace hi_can::addressing;

std::optional<PacketManager> packet_manager;

void handle_motor_speed_data(const Packet& packet);
void handle_motor_current_data(const Packet& packet);
void set_motor_speed(const excavation::bucket::controller::group& group,
                     const int16_t& speed);
void set_motor_current(const excavation::bucket::controller::group& group,
                       const uint16_t& current);
void register_motor_bank(const excavation::bucket::controller::group& group,
                         const uint8_t& speed_param);

std::optional<MotorBank> motor_bank_1;
std::optional<MotorBank> motor_bank_2;
std::optional<MotorBank> motor_bank_3;

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
    motor_bank_1.emplace(DRIVER_1_PINS,
                         DRIVER_2_PINS,
                         BANK_1_CURRENT_LIMIT,
                         BANK_1_CURRENT_SENSE,
                         BANK_1_FAULT);
    motor_bank_2.emplace(DRIVER_3_PINS,
                         DRIVER_4_PINS,
                         BANK_2_CURRENT_LIMIT,
                         BANK_2_CURRENT_SENSE,
                         BANK_2_FAULT);
    motor_bank_3.emplace(DRIVER_5_PINS,
                         DRIVER_6_PINS,
                         BANK_3_CURRENT_LIMIT,
                         BANK_3_CURRENT_SENSE,
                         BANK_3_FAULT);

    pinMode(MAGNET_PIN, OUTPUT);
    digitalWrite(MAGNET_PIN, LOW);

    auto& interface = TwaiInterface::get_instance(std::make_pair(bsp::CAN_TX_PIN, bsp::CAN_RX_PIN), 0,
                                                  filter_t{
                                                      .address = static_cast<flagged_address_t>(DEVICE_ADDRESS),
                                                      .mask = DEVICE_MASK,
                                                  });
    packet_manager.emplace(interface);

    using namespace excavation::bucket::controller;
    const std::vector<group> actuator_groups = {
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

    for (const auto& group : actuator_groups)
        register_motor_bank(group, static_cast<uint8_t>(actuator_parameter::SPEED));
    register_motor_bank(group::MAGNET,
                        static_cast<uint8_t>(magnet_parameter::ROTATE_SPEED));

    using namespace parameters::excavation::bucket::controller;
    packet_manager->set_transmission_config(
        static_cast<flagged_address_t>(
            standard_address_t{DEVICE_ADDRESS,
                               static_cast<uint8_t>(group::BANK_1),
                               static_cast<uint8_t>(bank_parameter::STATUS)}),
        {
            .generator = [=]()
            {
                return current_t{static_cast<uint16_t>(std::clamp(motor_bank_1->get_average_current() * 1000,
                                                                  static_cast<float>(std::numeric_limits<uint16_t>::min()),
                                                                  static_cast<float>(std::numeric_limits<uint16_t>::max())))}
                    .serialize_data();
            },
            .interval = 100ms,
            .should_transmit_immediately = true,
        });
    packet_manager->set_callback(
        filter_t{static_cast<flagged_address_t>(
            standard_address_t{DEVICE_ADDRESS,
                               static_cast<uint8_t>(group::BANK_1),
                               static_cast<uint8_t>(bank_parameter::CURRENT_LIMIT)})},
        {
            .data_callback = handle_motor_current_data,
        });
    packet_manager->set_transmission_config(
        static_cast<flagged_address_t>(
            standard_address_t{DEVICE_ADDRESS,
                               static_cast<uint8_t>(group::BANK_2),
                               static_cast<uint8_t>(bank_parameter::STATUS)}),
        {
            .generator = [=]()
            {
                return current_t{static_cast<uint16_t>(std::clamp(motor_bank_2->get_average_current() * 1000,
                                                                  static_cast<float>(std::numeric_limits<uint16_t>::min()),
                                                                  static_cast<float>(std::numeric_limits<uint16_t>::max())))}
                    .serialize_data();
            },
            .interval = 100ms,
            .should_transmit_immediately = true,
        });
    packet_manager->set_callback(
        filter_t{static_cast<flagged_address_t>(
            standard_address_t{DEVICE_ADDRESS,
                               static_cast<uint8_t>(group::BANK_2),
                               static_cast<uint8_t>(bank_parameter::CURRENT_LIMIT)})},
        {
            .data_callback = handle_motor_current_data,
        });
    packet_manager->set_transmission_config(
        static_cast<flagged_address_t>(
            standard_address_t{DEVICE_ADDRESS,
                               static_cast<uint8_t>(group::BANK_3),
                               static_cast<uint8_t>(bank_parameter::STATUS)}),
        {
            .generator = [=]()
            {
                return current_t{static_cast<uint16_t>(std::clamp(motor_bank_3->get_average_current() * 1000,
                                                                  static_cast<float>(std::numeric_limits<uint16_t>::min()),
                                                                  static_cast<float>(std::numeric_limits<uint16_t>::max())))}
                    .serialize_data();
            },
            .interval = 100ms,
            .should_transmit_immediately = true,
        });
    packet_manager->set_callback(
        filter_t{static_cast<flagged_address_t>(
            standard_address_t{DEVICE_ADDRESS,
                               static_cast<uint8_t>(group::BANK_3),
                               static_cast<uint8_t>(bank_parameter::CURRENT_LIMIT)})},
        {
            .data_callback = handle_motor_current_data,
        });
}

void loop()
{
    packet_manager->handle();
    delay(1);
}

void handle_motor_speed_data(const Packet& packet)
{
    using namespace excavation::bucket::controller;
    using namespace hi_can::parameters::excavation::bucket::controller;
    try
    {
        standard_address_t address{packet.get_address().address};
        set_motor_speed(
            static_cast<group>(standard_address_t(packet.get_address().address).group),
            speed_t{packet.get_data()}.value);
    }
    catch (const std::exception& e)
    {
        printf(std::format("Failed to parse speed packet: {}\n", e.what()).c_str());
    }
}
void handle_motor_current_data(const Packet& packet)
{
    using namespace excavation::bucket::controller;
    using namespace hi_can::parameters::excavation::bucket::controller;
    try
    {
        standard_address_t address{packet.get_address().address};
        set_motor_current(static_cast<group>(
                              standard_address_t(packet.get_address().address).group),
                          current_t{packet.get_data()}.value);
    }
    catch (const std::exception& e)
    {
        printf(std::format("Failed to parse current packet: {}\n", e.what()).c_str());
    }
}

void set_motor_speed(const excavation::bucket::controller::group& group, const int16_t& speed)
{
    using namespace excavation::bucket::controller;
    // printf(std::format("Setting motor (group) {:#x} to speed {}\n",
    //                    static_cast<uint8_t>(group), speed)
    //            .c_str());
    switch (group)
    {
    case group::LIFT_BOTH:
        set_motor_speed(group::LIFT_LEFT, speed);
        set_motor_speed(group::LIFT_RIGHT, speed);
        break;
    case group::LIFT_LEFT:
        motor_bank_1->set_speed_a(speed);
        break;
    case group::LIFT_RIGHT:
        motor_bank_1->set_speed_b(speed);
        break;
    case group::TILT_BOTH:
        motor_bank_3->set_speed_a(speed);
        break;
    case group::JAWS_BOTH:
        set_motor_speed(group::JAWS_LEFT, speed);
        set_motor_speed(group::JAWS_RIGHT, speed);
        break;
    case group::JAWS_LEFT:
        motor_bank_2->set_speed_a(speed);
        break;
    case group::JAWS_RIGHT:
        motor_bank_2->set_speed_b(speed);
        break;
    case group::MAGNET:
        motor_bank_3->set_speed_b(speed);
        break;
    default:
        break;
    }
}
void set_motor_current(const excavation::bucket::controller::group& group,
                       const uint16_t& current)
{
    using namespace excavation::bucket::controller;
    printf(std::format("Setting {:#x} current to {}mA\n", static_cast<uint8_t>(group), current).c_str());
    switch (group)
    {
    case group::BANK_1:
        motor_bank_1->set_current_limit(current / 1000.0f);
        break;
    case group::BANK_2:
        motor_bank_2->set_current_limit(current / 1000.0f);
        break;
    case group::BANK_3:
        motor_bank_3->set_current_limit(current / 1000.0f);
        break;
    default:
        break;
    }
}

void register_motor_bank(const excavation::bucket::controller::group& group,
                         const uint8_t& speed_param)
{
    const standard_address_t speed_address{DEVICE_ADDRESS,
                                           static_cast<uint8_t>(group),
                                           static_cast<uint8_t>(speed_param)};
    packet_manager->set_callback(filter_t{
                                     static_cast<flagged_address_t>(speed_address),
                                 },
                                 {
                                     .data_callback = handle_motor_speed_data,
                                     .timeout_callback = std::bind(set_motor_speed, group, (int16_t)0),
                                     .timeout = 200ms,
                                 });
}