#include "hi_can_parameter.hpp"

using namespace hi_can;
using namespace addressing;

using namespace std::chrono;
using namespace std::chrono_literals;

namespace hi_can::parameters::drive::vesc
{
#pragma pack(push, 1)
    struct raw_status_1_t {
        int32_t rpm;
        int16_t current;
        int16_t duty_cycle;
    };
    struct raw_status_2_t {
        int32_t ah;
        int32_t ah_charge;
    };
    struct raw_status_3_t {
        int32_t wh;
        int32_t wh_charge;
    };
    struct raw_status_4_t {
        int16_t temp_fet;
        int16_t temp_motor;
        int16_t current_in;
        int16_t pid_pos;
    };
    struct raw_status_5_t {
        int32_t tachometer;  // 4 bytes
        int16_t volts_in;    // 2 bytes
        uint16_t padding;    // 2 bytes | Should fix the VESC status feedback issue.
    };
    struct raw_status_6_t {
        int16_t adc1;
        int16_t adc2;
        int16_t adc3;
        int16_t ppm;
    };
#pragma pack(pop)

    void status_1_t::deserialize_data(const std::vector<uint8_t>& serialized_data) {
        SimpleSerializable<raw_status_1_t> raw_data(serialized_data);
        rpm = static_cast<int32_t>(ntohl(raw_data.rpm));
        current = static_cast<int16_t>(ntohs(raw_data.current)) / 10.0;
        duty_cycle = static_cast<int16_t>(ntohs(raw_data.duty_cycle)) / 1000.0;
    }
    std::vector<uint8_t> status_1_t::serialize_data() {
        SimpleSerializable<raw_status_1_t> raw_data;
        raw_data.rpm = htonl(rpm);
        raw_data.current = htons(current) * 10.0;
        raw_data.duty_cycle = htons(duty_cycle) * 1000.0;
        return raw_data.serialize_data();
    }

    void status_2_t::deserialize_data(const std::vector<uint8_t>& serialized_data) {
        SimpleSerializable<raw_status_2_t> raw_data(serialized_data);
        ah = static_cast<int32_t>(ntohl(raw_data.ah)) / 10000.0;
        ah_charge = static_cast<int32_t>(ntohl(raw_data.ah_charge)) / 10000.0;
    }
    std::vector<uint8_t> status_2_t::serialize_data() {
        SimpleSerializable<raw_status_2_t> raw_data;
        raw_data.ah = htonl(ah) * 10000.0;
        raw_data.ah_charge = htonl(ah_charge) * 10000.0;
        return raw_data.serialize_data();
    }

    void status_3_t::deserialize_data(const std::vector<uint8_t>& serialized_data) {
        SimpleSerializable<raw_status_3_t> raw_data(serialized_data);
        wh = static_cast<int32_t>(ntohl(raw_data.wh)) / 10000.0;
        wh_charge = static_cast<int32_t>(ntohl(raw_data.wh_charge)) / 10000.0;
    }
    std::vector<uint8_t> status_3_t::serialize_data() {
        SimpleSerializable<raw_status_3_t> raw_data;
        raw_data.wh = htonl(wh) * 10000.0;
        raw_data.wh_charge = htonl(wh_charge) * 10000.0;
        return raw_data.serialize_data();
    }

    void status_4_t::deserialize_data(const std::vector<uint8_t>& serialized_data) {
        SimpleSerializable<raw_status_4_t> raw_data(serialized_data);
        temp_fet = ntohs(raw_data.temp_fet) / 10.0;
        temp_motor = ntohs(raw_data.temp_motor) / 10.0;
        current_in = ntohs(raw_data.current_in) / 10.0;
        pid_pos = ntohs(raw_data.pid_pos) / 50.0;
    }
    std::vector<uint8_t> status_4_t::serialize_data() {
        SimpleSerializable<raw_status_4_t> raw_data;
        raw_data.temp_fet = htons(temp_fet) * 10.0;
        raw_data.temp_motor = htons(temp_motor) * 10.0;
        raw_data.current_in = htons(current_in) * 10.0;
        raw_data.pid_pos = htons(pid_pos) * 50.0;
        return raw_data.serialize_data();
    }

    void status_5_t::deserialize_data(const std::vector<uint8_t>& serialized_data) {
        SimpleSerializable<raw_status_5_t> raw_data(serialized_data);
        tachometer = static_cast<int32_t>(ntohl(raw_data.tachometer)) / 6.0;
        volts_in = ntohs(raw_data.volts_in) / 10.0;
    }
    std::vector<uint8_t> status_5_t::serialize_data() {
        SimpleSerializable<raw_status_5_t> raw_data;
        // TODO: this ain't right, needs to  be multiplied before conversion
        raw_data.tachometer = static_cast<int32_t>(htonl(tachometer)) * 6.0;
        raw_data.volts_in = htons(volts_in) * 10.0;
        return raw_data.serialize_data();
    }

    void status_6_t::deserialize_data(const std::vector<uint8_t>& serialized_data) {
        SimpleSerializable<raw_status_6_t> raw_data(serialized_data);
        adc1 = ntohs(raw_data.adc1) / 1000.0;
        adc2 = ntohs(raw_data.adc2) / 1000.0;
        adc3 = ntohs(raw_data.adc3) / 1000.0;
        ppm = ntohs(raw_data.ppm) / 1000.0;
    }
    std::vector<uint8_t> status_6_t::serialize_data() {
        SimpleSerializable<raw_status_6_t> raw_data;
        raw_data.adc1 = htons(adc1) * 1000.0;
        raw_data.adc2 = htons(adc2) * 1000.0;
        raw_data.adc3 = htons(adc3) * 1000.0;
        raw_data.ppm = htons(ppm) * 1000.0;
        return raw_data.serialize_data();
    }

    VescParameterGroup::VescParameterGroup(uint8_t vesc_id, steady_clock::duration transmission_interval)
        : _vesc_id(vesc_id) {
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

namespace hi_can::parameters::post_landing::arm::rmd_servo
{
    namespace send_message
    {

#pragma pack(push, 1)
        struct torque_raw_t {
            command_t _command = command_t::SET_TORQUE_CLOSED_LOOP;
            uint8_t _reserved1[3] = {};
            int16_t torque;
            uint16_t _reserved2 = 0;
        };
        struct speed_raw_t {
            command_t _command = command_t::SET_SPEED_CLOSED_LOOP;
            uint8_t _reserved[3] = {};
            int32_t speed = 0;
        };
        struct position_raw_t {
            position_t::position_command_t position_command;
            uint8_t _reserved = 0;
            uint16_t speed_limit;
            int32_t position_control;
        };
        struct single_turn_position_raw_t {
            command_t _command = command_t::SET_SINGLE_TURN_POSITION;
            single_turn_position_t::rotation_direction_t rotation_direction;
            uint16_t speed_limit;
            uint16_t position_control;
            uint16_t _reserved = 0;
        };
        struct action_command_raw_t {
            action_command_t::command_id_t command;
            uint8_t _reserved[7] = {};
        };
        struct function_control_raw_t {
            command_t _command = command_t::FUNCTION_CONTROL;
            function_control_t::function_index_t function_index;
            uint16_t _reserved = 0;
            uint32_t input_value;
        };
        struct active_reply_raw_t {
            command_t _command = command_t::ACTIVE_REPLY_FUNCTION;
            active_reply_t::reply_t reply_command;
            uint8_t enable;
            uint16_t reply_interval;
            uint8_t _reserved[3] = {};
        };
        struct zero_offset_raw_t {
            command_t _command;
            uint8_t _reserved[3] = {};
            uint32_t zero_bias = 0;
        };
        struct can_id_raw_t {
            command_t _command = command_t::SET_CAN_ID;
            uint8_t _reserved = 0;
            uint8_t read = 0;
            uint8_t _reserved2[3] = {};
            uint16_t can_id = 0;
        };
#pragma pack(pop)

        std::vector<uint8_t> torque_t::serialize_data() {
            SimpleSerializable<torque_raw_t> raw_data;
            raw_data.torque = torque * 100 / 0.12;  // 0.01 A/LSB, 0.12 Nm/A
            return raw_data.serialize_data();
        }
        std::vector<uint8_t> speed_t::serialize_data() {
            SimpleSerializable<speed_raw_t> raw_data;
            raw_data.speed = speed * 100;  // 0.01 dps/LSB
            return raw_data.serialize_data();
        }
        std::vector<uint8_t> position_t::serialize_data() {
            SimpleSerializable<position_raw_t> raw_data;
            raw_data.position_command = position_command;
            raw_data.speed_limit = speed_limit;
            raw_data.position_control = position_control * 100;  // 0.01 degrees/LSB
            return raw_data.serialize_data();
        }
        std::vector<uint8_t> single_turn_position_t::serialize_data() {
            SimpleSerializable<single_turn_position_raw_t> raw_data;
            raw_data.rotation_direction = rotation_direction;
            raw_data.speed_limit = speed_limit;                  // 1 dps/LSB
            raw_data.position_control = position_control * 100;  // 0.01 degrees/LSB
            return raw_data.serialize_data();
        }
        std::vector<uint8_t> action_command_t::serialize_data() {
            SimpleSerializable<action_command_raw_t> raw_data;
            raw_data.command = command;  // Empty message other than the command
            return raw_data.serialize_data();
        }
        std::vector<uint8_t> function_control_t::serialize_data() {
            SimpleSerializable<function_control_raw_t> raw_data;
            raw_data.function_index = function_index;
            raw_data.input_value = input_value;
            return raw_data.serialize_data();
        }
        std::vector<uint8_t> active_reply_t::serialize_data() {
            SimpleSerializable<active_reply_raw_t> raw_data;
            raw_data.reply_command = reply_command;
            raw_data.enable = uint8_t(enable);
            raw_data.reply_interval = reply_interval_ms / 10;  // 10ms/LSB
            return raw_data.serialize_data();
        }
        std::vector<uint8_t> zero_offset_t::serialize_data() {
            SimpleSerializable<zero_offset_raw_t> raw_data;
            raw_data._command = command;
            raw_data.zero_bias = zero_bias;
            return raw_data.serialize_data();
        }
        std::vector<uint8_t> can_id_t::serialize_data() {
            SimpleSerializable<can_id_raw_t> raw_data;
            raw_data.read = read ? 1 : 0;
            raw_data.can_id = can_id;
            return raw_data.serialize_data();
        }

    }
    namespace receive_message
    {

#pragma pack(push, 1)
        struct status_1_raw_t {
            command_t _command;  // Should always be 0x9A (status 1)
            int8_t motor_temperature;
            uint8_t _reserved;
            brake_control_t brake_status;
            uint16_t voltage;
            error_t error_status;
        };
        struct status_2_raw_t {
            status_2_t::status_2_command_t command;
            int8_t motor_temperature;
            int16_t torque_current;
            int16_t motor_speed;
            int16_t motor_angle;
        };
        struct status_3_raw_t {
            command_t _command;  // Should always be 0x9D (status 3)
            uint8_t motor_temperature;
            uint16_t phase_a_current;
            uint16_t phase_b_current;
            uint16_t phase_c_current;
        };
        struct single_turn_motor_status_raw_t {
            command_t _command;  // Should always be 0xA6 (single turn position)
            uint8_t motor_temperature = 0;
            uint16_t torque_current = 0;
            uint16_t motor_speed = 0;
            uint16_t motor_encoder = 0;
        };
        struct empty_raw_t {
            empty_t::empty_command_t command = {};
            uint8_t _reserved[7] = {};
        };
        struct zero_offset_raw_t {
            command_t _command;
            uint8_t _reserved[3] = {};
            uint32_t zero_bias = 0;
        };
        struct data_response_raw_t {
            command_t _command;
            uint8_t _reserved[3] = {};
            uint32_t data = 0;
        };
        struct can_id_raw_t {
            command_t _command = command_t::SET_CAN_ID;
            uint8_t _reserved = 0;
            uint8_t read = 0;
            uint8_t _reserved2[3] = {};
            uint16_t can_id = 0;
        };
#pragma pack(pop)

        void status_1_t::deserialize_data(const std::vector<uint8_t>& serialized_data) {
            SimpleSerializable<status_1_raw_t> raw_data(serialized_data);
            motor_temperature = raw_data.motor_temperature;  // 1 degree Celsius/LSB
            brake_status = raw_data.brake_status;
            voltage = raw_data.voltage / 10.0;  // 0.1V/LSB
        }
        void status_2_t::deserialize_data(const std::vector<uint8_t>& serialized_data) {
            SimpleSerializable<status_2_raw_t> raw_data(serialized_data);
            command = raw_data.command;
            motor_temperature = raw_data.motor_temperature;  // 1 degree Celsius/LSB
            torque_current = raw_data.torque_current / 100;  // 0.01 A/LSB
            motor_speed = raw_data.motor_speed;              // 1 dps/LSB
            motor_angle = raw_data.motor_angle;              // 1 degree/LSB
        }
        void status_3_t::deserialize_data(const std::vector<uint8_t>& serialized_data) {
            SimpleSerializable<status_3_raw_t> raw_data(serialized_data);
            motor_temperature = raw_data.motor_temperature;    // 1 degree Celsius/LSB
            phase_a_current = raw_data.phase_a_current / 100;  // 0.01 A/LSB
            phase_b_current = raw_data.phase_b_current / 100;  // 0.01 A/LSB
            phase_c_current = raw_data.phase_c_current / 100;  // 0.01 A/LSB
        }
        void single_turn_motor_status_t::deserialize_data(const std::vector<uint8_t>& serialized_data) {
            SimpleSerializable<single_turn_motor_status_raw_t> raw_data(serialized_data);
            motor_temperature = raw_data.motor_temperature;  // 1 degree Celsius/LSB
            torque_current = raw_data.torque_current / 100;  // 0.01 A/LSB
            motor_speed = raw_data.motor_speed;              // 1 dps/LSB
            motor_encoder = raw_data.motor_encoder;          // TODO: Check the value range of this - determined by the number of bits of the encoder
        }
        void empty_t::deserialize_data(const std::vector<uint8_t>& serialized_data) {
            SimpleSerializable<empty_raw_t> raw_data(serialized_data);
            command = raw_data.command;
        }
        void zero_offset_t::deserialize_data(const std::vector<uint8_t>& serialized_data) {
            SimpleSerializable<zero_offset_raw_t> raw_data(serialized_data);
            command = raw_data._command;
            zero_bias = raw_data.zero_bias;
        }
        void data_response_t::deserialize_data(const std::vector<uint8_t>& serialized_data) {
            SimpleSerializable<data_response_raw_t> raw_data(serialized_data);
            command = raw_data._command;
            data = raw_data.data;
        }
        void can_id_t::deserialize_data(const std::vector<uint8_t>& serialized_data) {
            SimpleSerializable<can_id_raw_t> raw_data(serialized_data);
            read = raw_data.read != 0;
            can_id = raw_data.can_id - static_cast<uint16_t>(addressing::post_landing::arm::rmd_servo::message_type::RECEIVE);
        }
    }
    RmdParameterGroup::RmdParameterGroup(uint8_t servo_id)
        : _servo_id(servo_id) {
        using namespace addressing::post_landing::arm::rmd_servo;

        _callbacks.emplace_back(
            filter_t{
                .address = servo_address_t{message_type::RECEIVE, motor_id_t(servo_id)},
            },
            PacketManager::callback_config_t{
                .data_callback = [this](const Packet& packet)
                {
                    std::vector<uint8_t> raw_data = packet.get_data();
                    command_t command = command_t(raw_data.front());
                    this->_online = true;  // set motor online when a message is received

                    switch (command) {
                    case command_t::READ_MOTOR_STATUS_1:
                    {
                        receive_message::status_1_t status_1 = {};
                        status_1.deserialize_data(raw_data);
                        this->_motor_temperature = status_1.motor_temperature;
                        this->_brake_status = status_1.brake_status;
                        this->_voltage = status_1.voltage;
                        this->_error_status = status_1.error_status;
                        break;
                    }
                    case command_t::READ_MOTOR_STATUS_2:
                    {
                        receive_message::status_2_t status_2 = {};
                        status_2.deserialize_data(raw_data);
                        this->_motor_temperature = status_2.motor_temperature;
                        this->_torque_current = status_2.torque_current;
                        this->_motor_speed = status_2.motor_speed;
                        this->_motor_angle = status_2.motor_angle;
                        break;
                    }
                    case command_t::READ_MOTOR_STATUS_3:
                    {
                        receive_message::status_3_t status_3 = {};
                        status_3.deserialize_data(raw_data);
                        this->_motor_temperature = status_3.motor_temperature;
                        this->_phase_a_current = status_3.phase_a_current;
                        this->_phase_b_current = status_3.phase_b_current;
                        this->_phase_c_current = status_3.phase_c_current;
                        break;
                    }
                    case command_t::READ_MULTI_TURN_POSITION:
                    {
                        receive_message::data_response_t data_response = {};
                        data_response.deserialize_data(raw_data);
                        this->_motor_angle = data_response.data;
                        break;
                    }
                    case command_t::MOTOR_STOP:
                    case command_t::MOTOR_SHUTDOWN:
                    case command_t::SYSTEM_BRAKE_RELEASE:
                    case command_t::SYSTEM_BRAKE_LOCK:
                        // These don't have any data sent back from the servos, so do nothing
                        break;
                    default:
                        break;
                    }
                    // switch (command)
                    // {
                    // case command_t::READ_MOTOR_STATUS_1:
                    // {
                    //     receive_message::status_1_t status_1 = {};
                    //     status_1.deserialize_data(raw_data);
                    //     this->_motor_temperature = status_1.motor_temperature;
                    //     this->_brake_status = status_1.brake_status;
                    //     this->_voltage = status_1.voltage;
                    //     this->_error_status = status_1.error_status;
                    //     break;
                    // }
                    // case command_t::SET_TORQUE_CLOSED_LOOP:
                    // case command_t::SET_SPEED_CLOSED_LOOP:
                    // case command_t::SET_ABSOLUTE_POSITION_CLOSED_LOOP:
                    // case command_t::SET_INCREMENTAL_POSITION_CLOSED_LOOP:
                    // case command_t::READ_MOTOR_STATUS_2:
                    // {
                    //     receive_message::status_2_t status_2 = {};
                    //     status_2.deserialize_data(raw_data);
                    //     this->_motor_temperature = status_2.motor_temperature;
                    //     this->_torque_current = status_2.torque_current;
                    //     this->_motor_speed = status_2.motor_speed;
                    //     this->_motor_angle = status_2.motor_angle;
                    //     break;
                    // }
                    // case command_t::READ_MOTOR_STATUS_3:
                    // {
                    //     receive_message::status_3_t status_3 = {};
                    //     status_3.deserialize_data(raw_data);
                    //     this->_motor_temperature = status_3.motor_temperature;
                    //     this->_phase_a_current = status_3.phase_a_current;
                    //     this->_phase_b_current = status_3.phase_b_current;
                    //     this->_phase_c_current = status_3.phase_c_current;
                    //     break;
                    // }
                    // case command_t::MOTOR_STOP:
                    // case command_t::MOTOR_SHUTDOWN:
                    // case command_t::SYSTEM_BRAKE_RELEASE:
                    // case command_t::SYSTEM_BRAKE_LOCK:
                    //     // These don't have any data sent back from the servos, so do nothing
                    //     break;
                    // default:
                    //     break;
                    // }
                },
            });
    }
    std::vector<std::string> RmdParameterGroup::check_errors() {
        std::vector<std::string> errors = {};
        if (uint16_t(_error_status) & uint16_t(error_t::ENCODER_CALIBRATION)) {
            errors.emplace_back("Encoder calibration");
        }
        if (uint16_t(_error_status) & uint16_t(error_t::OVER_TEMPERATURE)) {
            errors.emplace_back("Over temperature");
        }
        if (uint16_t(_error_status) & uint16_t(error_t::SPEEDING)) {
            errors.emplace_back("Speeding");
        }
        if (uint16_t(_error_status) & uint16_t(error_t::CALIBRATION_PARAMETER_WRITE)) {
            errors.emplace_back("Calibration parameter write");
        }
        if (uint16_t(_error_status) & uint16_t(error_t::POWER_OVERRUN)) {
            errors.emplace_back("Power overrun");
        }
        if (uint16_t(_error_status) & uint16_t(error_t::OVER_CURRENT)) {
            errors.emplace_back("Over current");
        }
        if (uint16_t(_error_status) & uint16_t(error_t::OVER_VOLTAGE)) {
            errors.emplace_back("Over voltage");
        }
        if (uint16_t(_error_status) & uint16_t(error_t::LOW_VOLTAGE)) {
            errors.emplace_back("Low voltage");
        }
        if (uint16_t(_error_status) & uint16_t(error_t::STALL)) {
            errors.emplace_back("Stall");
        }
        return errors;
    }
    bool RmdParameterGroup::is_online() {
        return _online;
    }
    void RmdParameterGroup::set_online(bool online) {
        _online = online;
    }
    int8_t RmdParameterGroup::get_temp() {
        return _motor_temperature;
    }
    brake_control_t RmdParameterGroup::get_brake_status() {
        return _brake_status;
    }
    double RmdParameterGroup::get_voltage() {
        return _voltage;
    }
    error_t RmdParameterGroup::get_error_status() {
        return _error_status;
    }
    double RmdParameterGroup::get_torque_current() {
        return _torque_current;
    }
    int16_t RmdParameterGroup::get_speed() {
        return _motor_speed;
    }
    int16_t RmdParameterGroup::get_angle() {
        return _motor_angle;
    }
    double RmdParameterGroup::get_phase_a_current() {
        return _phase_a_current;
    }
    double RmdParameterGroup::get_phase_b_current() {
        return _phase_b_current;
    }
    double RmdParameterGroup::get_phase_c_current() {
        return _phase_c_current;
    }
    std::vector<addressing::post_landing::arm::rmd_servo::motor_id_t> RmdParameterGroup::get_available_servos() {
        return _available_servos;
    }
}
namespace hi_can::parameters::post_landing::arm::control_board
{
    ControlBoardParameterGroup::ControlBoardParameterGroup(addressing::post_landing::arm::control_board::group servo_id)
        : _servo_id(servo_id) {
        const standard_address_t address(
            addressing::post_landing::SYSTEM_ID,
            addressing::post_landing::arm::SUBSYSTEM_ID,
            addressing::post_landing::arm::control_board::DEVICE_ID,
            static_cast<uint8_t>(servo_id));

        _callbacks.emplace_back(
            filter_t{
                .address = flagged_address_t(address),
                // Mask everything except for the parameter ID
                .mask = 0xFFFFFF00,
            },
            PacketManager::callback_config_t{
                .data_callback = [this](const Packet& packet)
                {
                    typedef hi_can::addressing::post_landing::arm::control_board::rsbl_parameters parameter_id_t;

                    const parameter_id_t parameter_id =
                        static_cast<parameter_id_t>(packet.get_address().address & 0x000000FF);
                    std::vector<uint8_t> raw_data = packet.get_data();
                    switch (parameter_id) {
                    case parameter_id_t::STATUS_1:
                    {
                        status_1_t status_1 = {};
                        status_1.deserialize_data(raw_data);
                        this->_position = status_1.position;
                        this->_status_1 = status_1;
                        break;
                    }
                    case parameter_id_t::STATUS_2:
                    {
                        status_2_t status_2 = {};
                        status_2.deserialize_data(raw_data);
                        this->_status_2 = status_2;
                        break;
                    }
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
        : _device_address(device_address) {
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
        : EscParameterGroup(group._device_address) {
        _speed = group._speed;
        _status = group._status;
    }

    std::vector<Packet> EscParameterGroup::get_startup_transmissions() const {
        // TODO: Fix namespace issues with legacy drive code
        std::vector<Packet> packets;
        // packets.emplace_back(
        //     flagged_address_t(
        //         address_t(_device_address,
        //                   static_cast<uint8_t>(mcb::groups::ESC),
        //                   static_cast<uint8_t>(esc::parameter::LIMITS)),
        //         true));
        return packets;
    }
}
namespace hi_can::parameters::legacy::power::control::power_bus
{
    PowerBusParameterGroup::PowerBusParameterGroup(const addressing::legacy::address_t& device_address,
        addressing::legacy::power::control::rcb::groups bus)
        : _device_address(device_address) {
        using namespace hi_can::addressing::legacy;

        _callbacks.emplace_back(
            filter_t{
                .address = static_cast<flagged_address_t>(
                    address_t(device_address,
                              static_cast<uint8_t>(bus),
                              static_cast<uint8_t>(addressing::legacy::power::control::power_bus::parameter::POWER_STATUS))) },
            PacketManager::callback_config_t{
                .data_callback = [this](const Packet& packet)
                {
                    _status.deserialize_data(packet.get_data());
                },
            });
    }
}