#include "hi_can_parameter.hpp"

using namespace hi_can;
using namespace parameters;
using namespace addressing;

using namespace std::chrono;
using namespace std::chrono_literals;

namespace hi_can::parameters::drive::vesc
{
#pragma pack(push, 1)
    struct raw_status_1_t
    {
        int32_t rpm;
        int16_t current;
        int16_t duty_cycle;
    };
    struct raw_status_2_t
    {
        int32_t ah;
        int32_t ah_charge;
    };
    struct raw_status_3_t
    {
        int32_t wh;
        int32_t wh_charge;
    };
    struct raw_status_4_t
    {
        int16_t temp_fet;
        int16_t temp_motor;
        int16_t current_in;
        int16_t pid_pos;
    };
    struct raw_status_5_t
    {
        int32_t tachometer;  // 4 bytes
        int16_t volts_in;    // 2 bytes
        uint16_t padding;    // 2 bytes | Should fix the VESC status feedback issue.
    };
    struct raw_status_6_t
    {
        int16_t adc1;
        int16_t adc2;
        int16_t adc3;
        int16_t ppm;
    };
#pragma pack(pop)

    void status_1_t::deserialize_data(const std::vector<uint8_t>& serialized_data)
    {
        SimpleSerializable<raw_status_1_t> raw_data(serialized_data);
        rpm = static_cast<int32_t>(ntohl(raw_data.rpm));
        current = static_cast<int16_t>(ntohs(raw_data.current)) / 10.0;
        duty_cycle = static_cast<int16_t>(ntohs(raw_data.duty_cycle)) / 1000.0;
    }
    std::vector<uint8_t> status_1_t::serialize_data()
    {
        SimpleSerializable<raw_status_1_t> raw_data;
        raw_data.rpm = htonl(rpm);
        raw_data.current = htons(current) * 10.0;
        raw_data.duty_cycle = htons(duty_cycle) * 1000.0;
        return raw_data.serialize_data();
    }

    void status_2_t::deserialize_data(const std::vector<uint8_t>& serialized_data)
    {
        SimpleSerializable<raw_status_2_t> raw_data(serialized_data);
        ah = static_cast<int32_t>(ntohl(raw_data.ah)) / 10000.0;
        ah_charge = static_cast<int32_t>(ntohl(raw_data.ah_charge)) / 10000.0;
    }
    std::vector<uint8_t> status_2_t::serialize_data()
    {
        SimpleSerializable<raw_status_2_t> raw_data;
        raw_data.ah = htonl(ah) * 10000.0;
        raw_data.ah_charge = htonl(ah_charge) * 10000.0;
        return raw_data.serialize_data();
    }

    void status_3_t::deserialize_data(const std::vector<uint8_t>& serialized_data)
    {
        SimpleSerializable<raw_status_3_t> raw_data(serialized_data);
        wh = static_cast<int32_t>(ntohl(raw_data.wh)) / 10000.0;
        wh_charge = static_cast<int32_t>(ntohl(raw_data.wh_charge)) / 10000.0;
    }
    std::vector<uint8_t> status_3_t::serialize_data()
    {
        SimpleSerializable<raw_status_3_t> raw_data;
        raw_data.wh = htonl(wh) * 10000.0;
        raw_data.wh_charge = htonl(wh_charge) * 10000.0;
        return raw_data.serialize_data();
    }

    void status_4_t::deserialize_data(const std::vector<uint8_t>& serialized_data)
    {
        SimpleSerializable<raw_status_4_t> raw_data(serialized_data);
        temp_fet = ntohs(raw_data.temp_fet) / 10.0;
        temp_motor = ntohs(raw_data.temp_motor) / 10.0;
        current_in = ntohs(raw_data.current_in) / 10.0;
        pid_pos = ntohs(raw_data.pid_pos) / 50.0;
    }
    std::vector<uint8_t> status_4_t::serialize_data()
    {
        SimpleSerializable<raw_status_4_t> raw_data;
        raw_data.temp_fet = htons(temp_fet) * 10.0;
        raw_data.temp_motor = htons(temp_motor) * 10.0;
        raw_data.current_in = htons(current_in) * 10.0;
        raw_data.pid_pos = htons(pid_pos) * 50.0;
        return raw_data.serialize_data();
    }

    void status_5_t::deserialize_data(const std::vector<uint8_t>& serialized_data)
    {
        SimpleSerializable<raw_status_5_t> raw_data(serialized_data);
        tachometer = static_cast<int32_t>(ntohl(raw_data.tachometer)) / 6.0;
        volts_in = ntohs(raw_data.volts_in) / 10.0;
    }
    std::vector<uint8_t> status_5_t::serialize_data()
    {
        SimpleSerializable<raw_status_5_t> raw_data;
        // TODO: this ain't right, needs to  be multiplied before conversion
        raw_data.tachometer = static_cast<int32_t>(htonl(tachometer)) * 6.0;
        raw_data.volts_in = htons(volts_in) * 10.0;
        return raw_data.serialize_data();
    }

    void status_6_t::deserialize_data(const std::vector<uint8_t>& serialized_data)
    {
        SimpleSerializable<raw_status_6_t> raw_data(serialized_data);
        adc1 = ntohs(raw_data.adc1) / 1000.0;
        adc2 = ntohs(raw_data.adc2) / 1000.0;
        adc3 = ntohs(raw_data.adc3) / 1000.0;
        ppm = ntohs(raw_data.ppm) / 1000.0;
    }
    std::vector<uint8_t> status_6_t::serialize_data()
    {
        SimpleSerializable<raw_status_6_t> raw_data;
        raw_data.adc1 = htons(adc1) * 1000.0;
        raw_data.adc2 = htons(adc2) * 1000.0;
        raw_data.adc3 = htons(adc3) * 1000.0;
        raw_data.ppm = htons(ppm) * 1000.0;
        return raw_data.serialize_data();
    }

    VescParameterGroup::VescParameterGroup(uint8_t vesc_id, steady_clock::duration transmission_interval)
        : _vesc_id(vesc_id)
    {
        using namespace addressing::drive::vesc;
        _transmissions.emplace_back(
            address_t(vesc_id, command_id::SET_RPM),
            PacketManager::transmission_config_t{
                .generator = [this]() -> auto
                {
                    return _set_rpm.serialize_data();
                },
                .interval = transmission_interval,
                .should_transmit_immediately = false,
            });

        _callbacks.emplace_back(
            filter_t{
                .address = static_cast<flagged_address_t>(address_t(vesc_id, command_id::STATUS_1)),
            },
            PacketManager::callback_config_t{
                .data_callback = [this](const Packet& packet)
                {
                    _status1.deserialize_data(packet.get_data());
                },
            });
        _callbacks.emplace_back(
            filter_t{
                .address = static_cast<flagged_address_t>(address_t(vesc_id, command_id::STATUS_2)),
            },
            PacketManager::callback_config_t{
                .data_callback = [this](const Packet& packet)
                {
                    _status2.deserialize_data(packet.get_data());
                },
            });
        _callbacks.emplace_back(
            filter_t{
                .address = static_cast<flagged_address_t>(address_t(vesc_id, command_id::STATUS_3)),
            },
            PacketManager::callback_config_t{
                .data_callback = [this](const Packet& packet)
                {
                    _status3.deserialize_data(packet.get_data());
                },
            });
        _callbacks.emplace_back(
            filter_t{
                .address = static_cast<flagged_address_t>(address_t(vesc_id, command_id::STATUS_4)),
            },
            PacketManager::callback_config_t{
                .data_callback = [this](const Packet& packet)
                {
                    _status4.deserialize_data(packet.get_data());
                },
            });
        _callbacks.emplace_back(
            filter_t{
                .address = static_cast<flagged_address_t>(address_t(vesc_id, command_id::STATUS_5)),
            },
            PacketManager::callback_config_t{
                .data_callback = [this](const Packet& packet)
                {
                    _status5.deserialize_data(packet.get_data());
                },
            });
        _callbacks.emplace_back(
            filter_t{
                .address = static_cast<flagged_address_t>(address_t(vesc_id, command_id::STATUS_6)),
            },
            PacketManager::callback_config_t{
                .data_callback = [this](const Packet& packet)
                {
                    _status6.deserialize_data(packet.get_data());
                },
            });
    }
}

namespace hi_can::parameters::post_landing::servo::rmd
{
    namespace send_message
    {

#pragma pack(push, 1)
        struct raw_torque_message_t
        {
            rmd_command _command = rmd_command::SET_TORQUE_CLOSED_LOOP;
            uint8_t _reserved1[3] = {};
            int16_t torque;
            uint16_t _reserved2 = 0;
        };
        struct raw_speed_message_t
        {
            rmd_command _command = rmd_command::SET_SPEED_CLOSED_LOOP;
            uint8_t _reserved[3] = {};
            int32_t speed = 0;
        };
        struct raw_position_message_t
        {
            position_message_t::position_command_t position_command;
            uint8_t _reserved = 0;
            uint16_t speed_limit;
            int32_t position_control;
        };
        struct raw_single_turn_position_message_t
        {
            rmd_command _command = rmd_command::SET_SINGLE_TURN_POSITION;
            single_turn_position_message_t::rotation_direction_t rotation_direction;
            uint16_t speed_limit;
            uint16_t position_control;
            uint16_t _reserved = 0;
        };
        struct raw_command_message_t
        {
            command_message_t::command_t command;
            uint8_t _reserved[7] = {};
        };
        struct raw_function_control_message_t
        {
            rmd_command _command = rmd_command::FUNCTION_CONTROL;
            function_control_message_t::function_index_t function_index;
            uint16_t _reserved = 0;
            uint32_t input_value;
        };
        struct raw_active_reply_message_t
        {
            rmd_command _command = rmd_command::ACTIVE_REPLY_FUNCTION;
            active_reply_message_t::reply_t reply_command;
            uint8_t enable;
            uint16_t reply_interval;
            uint8_t _reserved[3] = {};
        };
#pragma pack(pop)

        std::vector<uint8_t> torque_message_t::serialize_data()
        {
            SimpleSerializable<raw_torque_message_t> raw_data;
            raw_data.torque = torque * 100 / 0.12;  // 0.01 A/LSB, 0.12 Nm/A
            return raw_data.serialize_data();
        }
        std::vector<uint8_t> speed_message_t::serialize_data()
        {
            SimpleSerializable<raw_speed_message_t> raw_data;
            raw_data.speed = speed * 100;  // 0.01 dps/LSB
            return raw_data.serialize_data();
        }
        std::vector<uint8_t> position_message_t::serialize_data()
        {
            SimpleSerializable<raw_position_message_t> raw_data;
            raw_data.position_command = position_command;
            raw_data.speed_limit = speed_limit;
            raw_data.position_control = position_control * 100;  // 0.01 degrees/LSB
            return raw_data.serialize_data();
        }
        std::vector<uint8_t> single_turn_position_message_t::serialize_data()
        {
            SimpleSerializable<raw_single_turn_position_message_t> raw_data;
            raw_data.rotation_direction = rotation_direction;
            raw_data.speed_limit = speed_limit;                  // 1 dps/LSB
            raw_data.position_control = position_control * 100;  // 0.01 degrees/LSB
            return raw_data.serialize_data();
        }
        std::vector<uint8_t> command_message_t::serialize_data()
        {
            SimpleSerializable<raw_command_message_t> raw_data;
            raw_data.command = command;  // Empty message other than the command
            return raw_data.serialize_data();
        }
        std::vector<uint8_t> function_control_message_t::serialize_data()
        {
            SimpleSerializable<raw_function_control_message_t> raw_data;
            raw_data.function_index = function_index;
            raw_data.input_value = input_value;
            return raw_data.serialize_data();
        }
        std::vector<uint8_t> active_reply_message_t::serialize_data()
        {
            SimpleSerializable<raw_active_reply_message_t> raw_data;
            raw_data.reply_command = reply_command;
            raw_data.enable = uint8_t(enable);
            raw_data.reply_interval = reply_interval_ms / 10;  // 10ms/LSB
            return raw_data.serialize_data();
        }

    }
    namespace receive_message
    {

#pragma pack(push, 1)
        struct raw_motor_status_1_message_t
        {
            rmd_command _command;  // Should always be 0x9A (status 1)
            int8_t motor_temperature;
            uint8_t _reserved;
            shared::brake_control_t brake_status;
            uint16_t voltage;
            shared::error_t error_status;
        };
        struct raw_motor_status_2_message_t
        {
            motor_status_2_message_t::motor_status_2_command_t command;
            int8_t motor_temperature;
            int16_t torque_current;
            int16_t motor_speed;
            int16_t motor_angle;
        };
        struct raw_motor_status_3_message_t
        {
            rmd_command _command;  // Should always be 0x9D (status 3)
            uint8_t motor_temperature;
            uint16_t phase_a_current;
            uint16_t phase_b_current;
            uint16_t phase_c_current;
        };
        struct raw_single_turn_motor_status_message_t
        {
            rmd_command _command;  // Should always be 0xA6 (single turn position)
            uint8_t motor_temperature = 0;
            uint16_t torque_current = 0;
            uint16_t motor_speed = 0;
            uint16_t motor_encoder = 0;
        };
        struct raw_empty_message_t
        {
            empty_message_t::empty_command_t command = {};
            uint8_t _reserved[7] = {};
        };
#pragma pack(pop)

        void motor_status_1_message_t::deserialize_data(const std::vector<uint8_t>& serialized_data)
        {
            SimpleSerializable<raw_motor_status_1_message_t> raw_data(serialized_data);
            motor_temperature = raw_data.motor_temperature;  // 1 degree Celsius/LSB
            brake_status = raw_data.brake_status;
            voltage = raw_data.voltage / 10.0;  // 0.1V/LSB
        }
        void motor_status_2_message_t::deserialize_data(const std::vector<uint8_t>& serialized_data)
        {
            SimpleSerializable<raw_motor_status_2_message_t> raw_data(serialized_data);
            command = raw_data.command;
            motor_temperature = raw_data.motor_temperature;  // 1 degree Celsius/LSB
            torque_current = raw_data.torque_current / 100;  // 0.01 A/LSB
            motor_speed = raw_data.motor_speed;              // 1 dps/LSB
            motor_angle = raw_data.motor_angle;              // 1 degree/LSB
        }
        void motor_status_3_message_t::deserialize_data(const std::vector<uint8_t>& serialized_data)
        {
            SimpleSerializable<raw_motor_status_3_message_t> raw_data(serialized_data);
            motor_temperature = raw_data.motor_temperature;    // 1 degree Celsius/LSB
            phase_a_current = raw_data.phase_a_current / 100;  // 0.01 A/LSB
            phase_b_current = raw_data.phase_b_current / 100;  // 0.01 A/LSB
            phase_c_current = raw_data.phase_c_current / 100;  // 0.01 A/LSB
        }
        void single_turn_motor_status_message_t::deserialize_data(const std::vector<uint8_t>& serialized_data)
        {
            SimpleSerializable<raw_single_turn_motor_status_message_t> raw_data(serialized_data);
            motor_temperature = raw_data.motor_temperature;  // 1 degree Celsius/LSB
            torque_current = raw_data.torque_current / 100;  // 0.01 A/LSB
            motor_speed = raw_data.motor_speed;              // 1 dps/LSB
            motor_encoder = raw_data.motor_encoder;          // TODO: Check the value range of this - determined by the number of bits of the encoder
        }
        void empty_message_t::deserialize_data(const std::vector<uint8_t>& serialized_data)
        {
            SimpleSerializable<raw_empty_message_t> raw_data(serialized_data);
            command = raw_data.command;
        }
    }
    RmdParameterGroup::RmdParameterGroup(uint8_t servo_id) : _servo_id(servo_id)
    {
        _callbacks.emplace_back(
            filter_t{
                .address = addressing::post_landing::servo::rmd::servo_address_t{addressing::post_landing::servo::rmd::rmd_id::RECEIVE, addressing::post_landing::servo::rmd::motor_id(servo_id)},
            },
            PacketManager::callback_config_t{
                .data_callback = [this](const Packet& packet)
                {
                    std::vector<uint8_t> raw_data = packet.get_data();
                    rmd_command command = rmd_command(raw_data.front());
                    switch (command)
                    {
                    case rmd_command::READ_MOTOR_STATUS_1:
                    {
                        receive_message::motor_status_1_message_t status_1 = {};
                        status_1.deserialize_data(raw_data);
                        _motor_temperature = status_1.motor_temperature;
                        _brake_status = status_1.brake_status;
                        _voltage = status_1.voltage;
                        _error_status = status_1.error_status;
                        break;
                    }
                    case rmd_command::SET_TORQUE_CLOSED_LOOP:
                    case rmd_command::SET_SPEED_CLOSED_LOOP:
                    case rmd_command::SET_ABSOLUTE_POSITION_CLOSED_LOOP:
                    case rmd_command::SET_INCREMENTAL_POSITION_CLOSED_LOOP:
                    case rmd_command::READ_MOTOR_STATUS_2:
                    {
                        receive_message::motor_status_2_message_t status_2 = {};
                        status_2.deserialize_data(raw_data);
                        _motor_temperature = status_2.motor_temperature;
                        _torque_current = status_2.torque_current;
                        _motor_speed = status_2.motor_speed;
                        _motor_angle = status_2.motor_angle;
                        break;
                    }
                    case rmd_command::READ_MOTOR_STATUS_3:
                    {
                        receive_message::motor_status_3_message_t status_3 = {};
                        status_3.deserialize_data(raw_data);
                        _motor_temperature = status_3.motor_temperature;
                        _phase_a_current = status_3.phase_a_current;
                        _phase_b_current = status_3.phase_b_current;
                        _phase_c_current = status_3.phase_c_current;
                        break;
                    }
                    case rmd_command::MOTOR_STOP:
                    case rmd_command::MOTOR_SHUTDOWN:
                    case rmd_command::SYSTEM_BRAKE_RELEASE:
                    case rmd_command::SYSTEM_BRAKE_LOCK:
                        // These don't have any data sent back from the servos, so do nothing
                        break;
                    default:
                        break;
                    }
                },
            });
    }
}

namespace hi_can::parameters::legacy::drive::motors
{
    EscParameterGroup::EscParameterGroup(const addressing::legacy::address_t& device_address)
        : _device_address(device_address)
    {
        using namespace hi_can::addressing::legacy;
        using namespace hi_can::addressing::legacy::drive::motors;

        const auto speed_address = static_cast<flagged_address_t>(
            address_t(device_address,
                      static_cast<uint8_t>(mcb::groups::ESC),
                      static_cast<uint8_t>(esc::parameter::SPEED)));

        _transmissions.emplace_back(
            speed_address,
            PacketManager::transmission_config_t{
                .generator = [this]() -> auto
                {
                    return _speed.serialize_data();
                },
                .interval = 100ms,
                .should_transmit_immediately = false,
            });

        _callbacks.emplace_back(
            filter_t{
                .address = speed_address,
            },
            PacketManager::callback_config_t{
                .data_callback = [this](const Packet& packet)
                {
                    _speed.deserialize_data(packet.get_data());
                },
            });
        _callbacks.emplace_back(
            filter_t{
                .address = static_cast<flagged_address_t>(
                    address_t(device_address,
                              static_cast<uint8_t>(mcb::groups::ESC),
                              static_cast<uint8_t>(esc::parameter::STATUS))),
            },
            PacketManager::callback_config_t{
                .data_callback = [this](const Packet& packet)
                {
                    _status.deserialize_data(packet.get_data());
                },
            });
        _callbacks.emplace_back(
            filter_t{
                .address = static_cast<flagged_address_t>(
                    address_t(device_address,
                              static_cast<uint8_t>(mcb::groups::ESC),
                              static_cast<uint8_t>(esc::parameter::POSITION))),
            },
            PacketManager::callback_config_t{
                .data_callback = [this](const Packet& packet)
                {
                    _position = SimpleSerializable<wrapped_value_t<int64_t>>(packet.get_data()).value;
                },
            });
        _callbacks.emplace_back(
            filter_t{
                .address = static_cast<flagged_address_t>(
                    address_t(device_address,
                              static_cast<uint8_t>(mcb::groups::ESC),
                              static_cast<uint8_t>(esc::parameter::LIMITS))),
            },
            PacketManager::callback_config_t{
                .data_callback = [this](const Packet& packet)
                {
                    if (!_limits)
                        _limits = limits_t();
                    _limits->deserialize_data(packet.get_data());
                },
            });
    }
    EscParameterGroup::EscParameterGroup(const EscParameterGroup& group)
        : EscParameterGroup(group._device_address)
    {
        _speed = group._speed;
        _status = group._status;
    }

    std::vector<Packet> EscParameterGroup::get_startup_transmissions() const
    {
        using namespace hi_can::addressing::legacy;
        using namespace hi_can::addressing::legacy::drive::motors;

        std::vector<Packet> packets;
        packets.emplace_back(
            flagged_address_t{
                address_t(_device_address,
                          static_cast<uint8_t>(mcb::groups::ESC),
                          static_cast<uint8_t>(esc::parameter::LIMITS)),
                true,
            });
        return packets;
    }
}
namespace hi_can::parameters::legacy::power::control::power_bus
{
    PowerBusParameterGroup::PowerBusParameterGroup(const addressing::legacy::address_t& device_address,
                                                   addressing::legacy::power::control::rcb::groups bus)
        : _device_address(device_address)
    {
        using namespace hi_can::addressing::legacy;

        _callbacks.emplace_back(
            filter_t{
                .address = static_cast<flagged_address_t>(
                    address_t(device_address,
                              static_cast<uint8_t>(bus),
                              static_cast<uint8_t>(addressing::legacy::power::control::power_bus::parameter::POWER_STATUS)))},
            PacketManager::callback_config_t{
                .data_callback = [this](const Packet& packet)
                {
                    _status.deserialize_data(packet.get_data());
                },
            });
    }
}