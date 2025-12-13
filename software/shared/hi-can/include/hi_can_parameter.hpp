#pragma once

#include <netinet/in.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <optional>
#include <tuple>
#include <vector>

#include "hi_can.hpp"
#include "hi_can_address.hpp"

/**
 * @brief Declare a scaled int32_t type with a scaling factor
 *
 * Defined as a macro because we need the using declaration to bring the constructor into scope
 */
#define HI_CAN_PARAM_DECLARE_SCALED_INT32_T(_class_name, _scaling_factor) \
    struct _class_name : public scaled_int32_t<_scaling_factor>           \
    {                                                                     \
        using scaled_int32_t::scaled_int32_t;                             \
    };
/**
 * @brief Declare a scaled int16_t type with a scaling factor
 *
 * Defined as a macro because we need the using declaration to bring the constructor into scope
 */
#define HI_CAN_PARAM_DECLARE_SCALED_INT16_T(_class_name, _scaling_factor) \
    struct _class_name : public scaled_int16_t<_scaling_factor>           \
    {                                                                     \
        using scaled_int16_t::scaled_int16_t;                             \
    };

namespace hi_can::parameters
{
    class ParameterGroup
    {
    public:
        ParameterGroup() = default;

        // Delete copy/move constructors/assignments.
        // Since the the transmit/recieve callbacks are probably lambdas which capture `this`,
        // they likely also cannot be copied or moved.
        ParameterGroup(const ParameterGroup&) = default;
        ParameterGroup(ParameterGroup&&) = default;
        ParameterGroup& operator=(const ParameterGroup&) = delete;
        ParameterGroup& operator=(ParameterGroup&&) = delete;

        /// @brief List of callback configurations alongside their filters
        typedef std::vector<
            std::pair<addressing::filter_t, PacketManager::callback_config_t>>
            callback_configs_t;
        /// @brief List of transmission configurations alongside their flagged addresses
        typedef std::vector<
            std::pair<addressing::flagged_address_t, PacketManager::transmission_config_t>>
            transmission_configs_t;
        /// @brief Get the @ref PacketManager::callback_config_t "callbacks" for the data the group manages
        /// @return Group callback configs
        virtual const callback_configs_t& get_callbacks() const { return _callbacks; }
        /// @brief Get the @ref PacketManager::transmission_config_t "transmissions" the group manages
        /// @return Group transmission configs
        virtual const transmission_configs_t& get_transmissions() const { return _transmissions; }
        /// @brief Get a list of @ref Packet "Packets" to send upon adding the group
        /// @return Packets to transmit
        virtual std::vector<Packet> get_startup_transmissions() const { return {}; }

    protected:
        callback_configs_t _callbacks{};
        transmission_configs_t _transmissions{};
    };
    class Serializable
    {
    public:
        virtual std::vector<uint8_t> serialize_data() = 0;
    };
    class Deserializable
    {
    public:
        virtual void deserialize_data(const std::vector<uint8_t>& serialized_data) = 0;
    };
    class BidirectionalSerializable : public Serializable, public Deserializable
    {
    };
    template <double scaling_factor>
    struct scaled_int32_t : public BidirectionalSerializable
    {
        double value = 0;

        scaled_int32_t() = default;
        scaled_int32_t(const std::vector<uint8_t>& serialized_data)
        {
            deserialize_data(serialized_data);
        }
        void deserialize_data(const std::vector<uint8_t>& serialized_data) override
        {
            int32_t raw_data;
            if (serialized_data.size() != sizeof(raw_data))
                throw std::invalid_argument("Data size does not match");
            std::copy(serialized_data.begin(), serialized_data.end(), reinterpret_cast<uint8_t* const>(&raw_data));
            value = static_cast<int32_t>(ntohl(raw_data)) / scaling_factor;
        }
        std::vector<uint8_t> serialize_data() override
        {
            int32_t raw_data = htonl(static_cast<int32_t>(round(value * scaling_factor)));
            std::vector<uint8_t> data_buf;
            data_buf.resize(sizeof(raw_data));
            std::copy_n(reinterpret_cast<uint8_t* const>(&raw_data), sizeof(raw_data), data_buf.begin());
            return data_buf;
        }
    };
    template <double scaling_factor>
    struct scaled_int16_t : public BidirectionalSerializable
    {
        double value = 0;

        scaled_int16_t() = default;
        scaled_int16_t(double _value)
            : value(_value)
        {
        }
        scaled_int16_t(const std::vector<uint8_t>& serialized_data)
        {
            deserialize_data(serialized_data);
        }

        void deserialize_data(const std::vector<uint8_t>& serialized_data) override
        {
            int16_t raw_data;
            if (serialized_data.size() != sizeof(raw_data))
                throw std::invalid_argument("Data size does not match");
            std::copy(serialized_data.begin(), serialized_data.end(), reinterpret_cast<uint8_t* const>(&raw_data));
            value = static_cast<int16_t>(ntohs(raw_data)) / scaling_factor;
        }
        std::vector<uint8_t> serialize_data() override
        {
            int16_t raw_data = htons(static_cast<int16_t>(round(value * scaling_factor)));
            std::vector<uint8_t> data_buf;
            data_buf.resize(sizeof(raw_data));
            std::copy_n(reinterpret_cast<uint8_t* const>(&raw_data), sizeof(raw_data), data_buf.begin());
            return data_buf;
        }
    };
    template <typename T>
    struct wrapped_value_t
    {
        wrapped_value_t() = default;
        wrapped_value_t(T _value)
            : value(_value)
        {
        }
        T value{};
    };
    template <typename T>
    class SimpleSerializable : public BidirectionalSerializable, public T
    {
    public:
        using T::T;

        SimpleSerializable(const T& value)
        {
            static_cast<T&>(*this) = value;
        }
        SimpleSerializable(const std::vector<uint8_t>& serialized_data)
        {
            deserialize_data(serialized_data);
        }
        void deserialize_data(const std::vector<uint8_t>& serialized_data) override
        {
            if (serialized_data.size() != sizeof(T))
                throw std::invalid_argument("Data size does not match");
            std::copy(serialized_data.begin(), serialized_data.end(), reinterpret_cast<uint8_t* const>(static_cast<T*>(this)));
        }
        std::vector<uint8_t> serialize_data() override
        {
            std::vector<uint8_t> data_buf;
            data_buf.resize(sizeof(T));
            std::copy_n(reinterpret_cast<uint8_t* const>(static_cast<T*>(this)), sizeof(T), data_buf.begin());
            return data_buf;
        }
    };

    namespace drive
    {
        namespace vesc
        {
            // COMMAND STRUCTS
            HI_CAN_PARAM_DECLARE_SCALED_INT32_T(set_duty_t, 100000.0)
            HI_CAN_PARAM_DECLARE_SCALED_INT32_T(set_current_t, 1000.0)
            HI_CAN_PARAM_DECLARE_SCALED_INT32_T(set_brake_t, 1000.0)
            HI_CAN_PARAM_DECLARE_SCALED_INT32_T(set_rpm_t, 1.0)
            HI_CAN_PARAM_DECLARE_SCALED_INT32_T(set_pos_t, 1000000.0)
            HI_CAN_PARAM_DECLARE_SCALED_INT32_T(set_current_rel_t, 100000.0)
            HI_CAN_PARAM_DECLARE_SCALED_INT32_T(set_current_brake_rel_t, 100000.0)
            HI_CAN_PARAM_DECLARE_SCALED_INT32_T(set_current_handbrake_t, 1000.0)
            HI_CAN_PARAM_DECLARE_SCALED_INT32_T(set_current_handbrake_rel_t, 100000.0)

            // STATUS STRUCTS
            struct status_1_t : public BidirectionalSerializable
            {
                double rpm = 0;
                double current = 0;
                double duty_cycle = 0;

                void deserialize_data(const std::vector<uint8_t>& serialized_data) override;
                std::vector<uint8_t> serialize_data() override;
            };
            struct status_2_t : public BidirectionalSerializable
            {
                double ah = 0;
                double ah_charge = 0;

                void deserialize_data(const std::vector<uint8_t>& serialized_data) override;
                std::vector<uint8_t> serialize_data() override;
            };
            struct status_3_t : public BidirectionalSerializable
            {
                double wh = 0;
                double wh_charge = 0;

                void deserialize_data(const std::vector<uint8_t>& serialized_data) override;
                std::vector<uint8_t> serialize_data() override;
            };
            struct status_4_t : public BidirectionalSerializable
            {
                double temp_fet = 0;
                double temp_motor = 0;
                double current_in = 0;
                double pid_pos = 0;

                void deserialize_data(const std::vector<uint8_t>& serialized_data) override;
                std::vector<uint8_t> serialize_data() override;
            };
            struct status_5_t : public BidirectionalSerializable
            {
                double tachometer = 0;
                double volts_in = 0;

                void deserialize_data(const std::vector<uint8_t>& serialized_data) override;
                std::vector<uint8_t> serialize_data() override;
            };
            struct status_6_t : public BidirectionalSerializable
            {
                double adc1 = 0;
                double adc2 = 0;
                double adc3 = 0;
                double ppm = 0;

                void deserialize_data(const std::vector<uint8_t>& serialized_data) override;
                std::vector<uint8_t> serialize_data() override;
            };

            class VescParameterGroup : public ParameterGroup
            {
            public:
                VescParameterGroup(uint8_t vesc_id, std::chrono::steady_clock::duration transmission_interval = std::chrono::steady_clock::duration::zero());

                auto& get_status1() { return _status1; }
                auto& get_status2() { return _status2; }
                auto& get_status3() { return _status3; }
                auto& get_status4() { return _status4; }
                auto& get_status5() { return _status5; }
                auto& get_status6() { return _status6; }

                auto& get_set_rpm() { return _set_rpm; }

            private:
                uint8_t _vesc_id = 0;

                set_rpm_t _set_rpm;

                status_1_t _status1;
                status_2_t _status2;
                status_3_t _status3;
                status_4_t _status4;
                status_5_t _status5;
                status_6_t _status6;
            };
        }
    }
    namespace power
    {
        namespace bms
        {
            struct pack_status
            {
            };
        }
        namespace distribution
        {
            struct bus_status_t
            {
                enum class status_t : uint8_t
                {
                    OFF = 0,
                    ON,
                    PRECHARGING,
                    PRECHARGE_FAIL,
                    SWITCH_FAILED,
                    OVERLOAD,
                    FAULT,
                };
                status_t status_t;
                uint16_t voltage;  // in mV
                uint32_t current;  // in mA
            };
        }
    }
    namespace post_landing
    {
        namespace arm
        {
            namespace servo
            {
                enum class _rmd_command : uint8_t  // The first byte of every RMD servo message (send or receive) should be of these commands
                {
                    FUNCTION_CONTROL = 0x20,
                    READ_PID = 0x30,
                    WRITE_PID_RAM = 0x31,
                    WRITE_PID_ROM = 0x32,
                    WRITE_ACCELERATION = 0x43,
                    READ_MULTI_TURN_POSITION = 0x60,
                    READ_MULTI_TURN_ORIGINAL_POSITION = 0x61,
                    READ_MULTI_TURN_ZERO_OFFSET = 0x62,
                    WRITE_MULTI_TURN_ROM_AS_ZERO = 0x63,
                    WRITE_CURRENT_MULTI_TURN_ROM_AS_ZERO = 0x64,
                    READ_SYSTEM_OPERATING_MODE = 0x70,
                    SYSTEM_RESET = 0x76,
                    SYSTEM_BRAKE_RELEASE = 0x77,
                    SYSTEM_BRAKE_LOCK = 0x78,
                    SET_CAN_ID = 0x79,
                    MOTOR_SHUTDOWN = 0x80,
                    MOTOR_STOP = 0x81,
                    READ_SINGLE_TURN_ENCODER = 0x90,
                    READ_MULTI_TURN_ANGLE = 0x92,
                    READ_SINGLE_TURN_ANGLE = 0x94,

                    // Statuses
                    READ_MOTOR_STATUS_1 = 0x9A,
                    READ_MOTOR_STATUS_2 = 0x9C,
                    READ_MOTOR_STATUS_3 = 0x9D,

                    // CONTROL COMMANDS
                    SET_TORQUE_CLOSED_LOOP = 0xA1,
                    SET_SPEED_CLOSED_LOOP = 0xA2,
                    SET_ABSOLUTE_POSITION_CLOSED_LOOP = 0xA4,
                    SET_SINGLE_TURN_POSITION = 0xA6,
                    SET_INCREMENTAL_POSITION_CLOSED_LOOP = 0xA8,

                    READ_SYSTEM_RUNTIME = 0xB1,
                    READ_SYSTEM_SOFTWARE_DATE = 0xB2,
                    SET_COMMUNICATION_INTERRUPTION_PROTECTION_TIME = 0xB3,
                    SET_COMMUNICATION_BAUD_RATE = 0xB4,
                    READ_MOTOR_MODEL = 0xB5,
                    ACTIVE_REPLY_FUNCTION = 0xB6,
                };
                namespace send_message  // The servo takes one of these messages (with the send address), then sends the corresponding receive message with the same command and the receive address
                {
                    struct torque_message_t : Serializable
                    {
                        double torque = 0;
                        std::vector<uint8_t> serialize_data() override;
                    };

                    struct speed_message_t : Serializable
                    {
                        double speed = 0;
                        std::vector<uint8_t> serialize_data() override;
                    };

                    struct position_message_t : Serializable
                    {
                        enum class position_command_t : uint8_t
                        {
                            ABSOLUTE = uint8_t(_rmd_command::SET_ABSOLUTE_POSITION_CLOSED_LOOP),
                            INCREMENTAL = uint8_t(_rmd_command::SET_INCREMENTAL_POSITION_CLOSED_LOOP),
                        };
                        position_command_t position_command;
                        uint16_t speed_limit = 0;
                        double position_control = 0;
                        std::vector<uint8_t> serialize_data() override;
                    };

                    struct single_turn_position_message_t : Serializable
                    {
                        _rmd_command _command = _rmd_command::SET_SINGLE_TURN_POSITION;
                        enum class rotation_direction_t : uint8_t
                        {
                            CLOCKWISE = 0x00,
                            ANTICLOCKWISE = 0x01,
                        };
                        rotation_direction_t rotation_direction = {};
                        uint16_t speed_limit = 0;
                        double position_control = 0;
                        std::vector<uint8_t> serialize_data() override;
                    };

                    struct command_message_t : Serializable
                    {
                        enum class command_t : uint8_t
                        {
                            SHUTDOWN = uint8_t(_rmd_command::MOTOR_SHUTDOWN),
                            STOP = uint8_t(_rmd_command::MOTOR_STOP),
                            BRAKE_RELEASE = uint8_t(_rmd_command::SYSTEM_BRAKE_RELEASE),
                            BRAKE_LOCK = uint8_t(_rmd_command::SYSTEM_BRAKE_LOCK),
                            STATUS_1 = uint8_t(_rmd_command::READ_MOTOR_STATUS_1),
                            STATUS_2 = uint8_t(_rmd_command::READ_MOTOR_STATUS_2),
                            STATUS_3 = uint8_t(_rmd_command::READ_MOTOR_STATUS_3),
                        };
                        command_t command = {};
                        std::vector<uint8_t> serialize_data() override;
                    };

                    struct _function_control_t
                    {
                        _rmd_command _command = _rmd_command::FUNCTION_CONTROL;
                        enum class function_index_t : uint8_t
                        {
                            CLEAR_MULTI_TURN = 0x01,
                            CANID_FILTER_ENABLE = 0x02,
                            ERROR_TRANSMISSION_ENABLE = 0x03,
                            MULTI_TURN_SAVE_ENABLE = 0x04,
                            SET_CANID = 0x05,
                            SET_MAXIMUM_POSITIVE_ANGLE = 0x06,
                            SET_MAXIMUM_NEGATIVE_ANGLE = 0x07,
                        };
                        function_index_t function_index = {};
                        uint16_t _reserved = 0;
                        uint32_t input_value = 0;
                    };
                    typedef SimpleSerializable<_function_control_t> function_control_t;
                }
                namespace receive_message
                {
                    struct motor_status_1_message_t : Deserializable
                    {
                        int8_t motor_temperature = 0;
                        enum class brake_control_t : uint8_t
                        {
                            BRAKE_LOCK = 0x00,
                            BRAKE_RELEASE = 0x01,
                        };
                        brake_control_t brake_control = {};
                        double voltage = 0;
                        enum class error_t : uint16_t
                        {
                            STALL = 0x0002,
                            LOW_VOLTAGE = 0x0004,
                            OVER_VOLTAGE = 0x0008,
                            OVER_CURRENT = 0x0010,
                            POWER_OVERRUN = 0x0040,
                            CALIBRATION_PARAMETER_WRITE = 0x0080,
                            SPEEDING = 0x0100,
                            OVER_TEMPERATURE = 0x1000,
                            ENCODER_CALIBRATION = 0x2000,
                        };
                        error_t error_status = {};
                        void deserialize_data(const std::vector<uint8_t>& serialized_data) override;
                    };

                    struct motor_status_2_message_t : Deserializable
                    {
                        enum class motor_status_2_command_t
                        {
                            STATUS_2 = uint8_t(_rmd_command::READ_MOTOR_STATUS_2),
                            TORQUE = uint8_t(_rmd_command::SET_TORQUE_CLOSED_LOOP),
                            SPEED = uint8_t(_rmd_command::SET_SPEED_CLOSED_LOOP),
                            ABSOLUTE_POSITION = uint8_t(_rmd_command::SET_ABSOLUTE_POSITION_CLOSED_LOOP),
                            INCREMENTAL_POSITION = uint8_t(_rmd_command::SET_INCREMENTAL_POSITION_CLOSED_LOOP),
                        };
                        motor_status_2_command_t command = {};
                        int8_t motor_temperature = 0;
                        double torque_current = 0;
                        int16_t motor_speed = 0;
                        int16_t motor_angle = 0;
                        void deserialize_data(const std::vector<uint8_t>& serialized_data) override;
                    };

                    struct motor_status_3_message_t : Deserializable
                    {
                        int8_t motor_temperature = 0;
                        double phase_a_current = 0;
                        double phase_b_current = 0;
                        double phase_c_current = 0;
                        void deserialize_data(const std::vector<uint8_t>& serialized_data) override;
                    };

                    struct single_turn_motor_status_message_t : Deserializable
                    {
                        _rmd_command command = _rmd_command::SET_SINGLE_TURN_POSITION;
                        int8_t motor_temperature = 0;
                        double torque_current = 0;
                        int16_t motor_speed = 0;
                        uint16_t motor_encoder = 0;
                        void deserialize_data(const std::vector<uint8_t>& serialized_data) override;
                    };

                    struct empty_message_t : Deserializable
                    {
                        enum class empty_command_t
                        {
                            STOP = uint8_t(_rmd_command::MOTOR_STOP),
                            SHUTDOWN = uint8_t(_rmd_command::MOTOR_SHUTDOWN),
                            BRAKE_RELEASE = uint8_t(_rmd_command::SYSTEM_BRAKE_RELEASE),
                            BRAKE_LOCK = uint8_t(_rmd_command::SYSTEM_BRAKE_LOCK),
                        };
                        empty_command_t command = {};
                        void deserialize_data(const std::vector<uint8_t>& serialized_data) override;
                    };

                    struct _function_message_t
                    {
                        _rmd_command _command = _rmd_command::FUNCTION_CONTROL;
                        enum class function_index_t : uint8_t
                        {
                            CLEAR_MULTI_TURN = 0x01,
                            CANID_FILTER_ENABLE = 0x02,
                            ERROR_TRANSMISSION_ENABLE = 0x03,
                            MULTI_TURN_SAVE_ENABLE = 0x04,
                            SET_CANID = 0x05,
                            SET_MAXIMUM_POSITIVE_ANGLE = 0x06,
                            SET_MAXIMUM_NEGATIVE_ANGLE = 0x07,
                        };
                        function_index_t function_index = {};
                        uint16_t _reserved = 0;
                        uint32_t input_value = 0;
                    };
                    typedef SimpleSerializable<_function_message_t> function_message_t;
                }
            }

        }
    }
    namespace excavation
    {
        namespace bucket
        {
            namespace controller
            {
                typedef SimpleSerializable<wrapped_value_t<int16_t>> speed_t;
                typedef SimpleSerializable<wrapped_value_t<uint16_t>> current_t;
                typedef SimpleSerializable<wrapped_value_t<bool>> magnet_t;
            }
        }
    }
    namespace shared
    {
        namespace elevator
        {
            namespace elevator
            {
                namespace motor
                {
                    HI_CAN_PARAM_DECLARE_SCALED_INT16_T(speed_t, 1.0)
                }
            }
        }
    }

    namespace legacy
    {
        namespace drive
        {
            namespace motors
            {
#pragma pack(push, 1)
                enum class motor_direction : int8_t
                {
                    REVERSE = -1,
                    STOP = 0,
                    FORWARD = 1,
                };
                struct _speed_t
                {
                    bool enabled = false;
                    motor_direction direction = motor_direction::STOP;
                    int16_t speed = 0;
                };
                struct _status_t
                {
                    bool ready = false;
                    int16_t real_speed = 0;
                    int16_t real_current = 0;
                };
                struct _limits_t
                {
                    int16_t max_current = 0;
                    int16_t ramp_speed = 0;
                };
#pragma pack(pop)
                typedef SimpleSerializable<_speed_t> speed_t;
                typedef SimpleSerializable<_status_t> status_t;
                typedef SimpleSerializable<_limits_t> limits_t;
                class EscParameterGroup : public ParameterGroup
                {
                public:
                    EscParameterGroup(const addressing::legacy::address_t& device_address);

                    EscParameterGroup(const EscParameterGroup&);

                    std::vector<Packet> get_startup_transmissions() const override;

                    auto& get_speed() { return _speed; }
                    auto& get_status() { return _status; }
                    auto& get_position() { return _position; }
                    auto& get_limits() { return _position; }

                private:
                    addressing::legacy::address_t _device_address;
                    speed_t _speed{};
                    status_t _status{};
                    int64_t _position{};
                    std::optional<limits_t> _limits{};
                };
            }
        }
        namespace power
        {
            namespace control
            {
                namespace contactor
                {

#pragma pack(push, 1)
                    struct _control_t
                    {
                        bool immediate_shutdown : 1 = false;
                        uint8_t _reserved : 7 = 0;   // padding to make a full byte
                        uint8_t shutdown_timer = 0;  // if a non-0 value is received, shutdown in that many seconds
                    };
#pragma pack(pop)
                    typedef SimpleSerializable<_control_t> control_t;
                }
                namespace power_bus
                {
                    enum class power_status : uint8_t
                    {
                        OFF = 0,        // bus off
                        ON,             // bus on
                        PRECHARGING,    // bus is precharging
                        SHORT_CIRCUIT,  // precharging failed - short circuit
                        SWITCH_FAILED,  // main switch not turning on - estop?
                        OVERLOAD,       // software fuse triggered
                        FAULT,          // switch reporting fault
                    };
#pragma pack(push, 1)
                    struct _status_t
                    {
                        power_status status = power_status::OFF;
                        uint16_t voltage = 0;  // in mV
                        uint32_t current = 0;  // in mA
                    };
                    struct _immediate_control_t
                    {
                        bool bus_target_state : 1 = false;  // bus on/off state
                        bool clear_error : 1 = false;       // retry if an error has occurred
                        uint8_t _reserved : 6 = 0;          // padding to make a full byte
                    };
                    struct _scheduled_control_t
                    {
                        uint8_t bus_off_time = 0;  // if a non-0 value is received, turn off bus in that many seconds
                        uint8_t bus_on_time = 0;   // if a non-0 value is received, turn on bus in that many seconds
                    };
#pragma pack(pop)
                    typedef SimpleSerializable<_status_t> status_t;
                    typedef SimpleSerializable<_immediate_control_t> immediate_control_t;
                    typedef SimpleSerializable<_scheduled_control_t> scheduled_control_t;
                    class PowerBusParameterGroup : public ParameterGroup
                    {
                    public:
                        PowerBusParameterGroup(const addressing::legacy::address_t& device_address, addressing::legacy::power::control::rcb::groups bus);

                        auto& get_status() { return _status; }

                    private:
                        addressing::legacy::address_t _device_address;
                        status_t _status{};
                    };
                }
            }
        }
        namespace excavation
        {
            namespace bucket
            {
#pragma pack(push, 1)
                struct _motor_speed_t
                {
                    _motor_speed_t() = default;
                    _motor_speed_t(bool _enable, int16_t _speed)
                        : enable(_enable),
                          speed(_speed)
                    {
                    }
                    bool enable = 0;
                    int16_t speed = 0;
                };
#pragma pack(pop)
                typedef SimpleSerializable<_motor_speed_t> motor_speed_t;
            }
        }
    }
}