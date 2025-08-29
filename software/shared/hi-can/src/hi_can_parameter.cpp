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