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
        int16_t dutyCycle;
    };
    struct raw_status_2_t
    {
        int32_t ah;
        int32_t ahCharge;
    };
    struct raw_status_3_t
    {
        int32_t wh;
        int32_t whCharge;
    };
    struct raw_status_4_t
    {
        int16_t tempFet;
        int16_t tempMotor;
        int16_t currentIn;
        int16_t pidPos;
    };
    struct raw_status_5_t
    {
        int32_t tachometer;
        int16_t voltsIn;
    };
    struct raw_status_6_t
    {
        int16_t adc1;
        int16_t adc2;
        int16_t adc3;
        int16_t ppm;
    };
#pragma pack(pop)

    void status_1_t::deserializeData(const std::vector<uint8_t>& serializedData)
    {
        SimpleSerializable<raw_status_1_t> rawData(serializedData);
        rpm = ntohl(rawData.rpm);
        current = ntohs(rawData.current) / 10.0;
        dutyCycle = ntohs(rawData.dutyCycle) / 1000.0;
    }
    std::vector<uint8_t> status_1_t::serializeData()
    {
        SimpleSerializable<raw_status_1_t> rawData;
        rawData.rpm = htonl(rpm);
        rawData.current = htons(current) * 10.0;
        rawData.dutyCycle = htons(dutyCycle) * 1000.0;
        return rawData.serializeData();
    }

    void status_2_t::deserializeData(const std::vector<uint8_t>& serializedData)
    {
        SimpleSerializable<raw_status_2_t> rawData(serializedData);
        ah = ntohl(rawData.ah) / 10000.0;
        ahCharge = ntohl(rawData.ahCharge) / 10000.0;
    }
    std::vector<uint8_t> status_2_t::serializeData()
    {
        SimpleSerializable<raw_status_2_t> rawData;
        rawData.ah = htonl(ah) * 10000.0;
        rawData.ahCharge = htonl(ahCharge) * 10000.0;
        return rawData.serializeData();
    }

    void status_3_t::deserializeData(const std::vector<uint8_t>& serializedData)
    {
        SimpleSerializable<raw_status_3_t> rawData(serializedData);
        wh = ntohl(rawData.wh) / 10000.0;
        whCharge = ntohl(rawData.whCharge) / 10000.0;
    }
    std::vector<uint8_t> status_3_t::serializeData()
    {
        SimpleSerializable<raw_status_3_t> rawData;
        rawData.wh = htonl(wh) * 10000.0;
        rawData.whCharge = htonl(whCharge) * 10000.0;
        return rawData.serializeData();
    }

    void status_4_t::deserializeData(const std::vector<uint8_t>& serializedData)
    {
        SimpleSerializable<raw_status_4_t> rawData(serializedData);
        tempFet = ntohs(rawData.tempFet) / 10.0;
        tempMotor = ntohs(rawData.tempMotor) / 10.0;
        currentIn = ntohs(rawData.currentIn) / 10.0;
        pidPos = ntohs(rawData.pidPos) / 50.0;
    }
    std::vector<uint8_t> status_4_t::serializeData()
    {
        SimpleSerializable<raw_status_4_t> rawData;
        rawData.tempFet = htons(tempFet) * 10.0;
        rawData.tempMotor = htons(tempMotor) * 10.0;
        rawData.currentIn = htons(currentIn) * 10.0;
        rawData.pidPos = htons(pidPos) * 50.0;
        return rawData.serializeData();
    }

    void status_5_t::deserializeData(const std::vector<uint8_t>& serializedData)
    {
        SimpleSerializable<raw_status_5_t> rawData(serializedData);
        tachometer = ntohl(rawData.tachometer) / 6.0;
        voltsIn = ntohs(rawData.voltsIn) / 10.0;
    }
    std::vector<uint8_t> status_5_t::serializeData()
    {
        SimpleSerializable<raw_status_5_t> rawData;
        rawData.tachometer = htonl(tachometer) * 6.0;
        rawData.voltsIn = htons(voltsIn) * 10.0;
        return rawData.serializeData();
    }

    void status_6_t::deserializeData(const std::vector<uint8_t>& serializedData)
    {
        SimpleSerializable<raw_status_6_t> rawData(serializedData);
        adc1 = ntohs(rawData.adc1) / 1000.0;
        adc2 = ntohs(rawData.adc2) / 1000.0;
        adc3 = ntohs(rawData.adc3) / 1000.0;
        ppm = ntohs(rawData.ppm) / 1000.0;
    }
    std::vector<uint8_t> status_6_t::serializeData()
    {
        SimpleSerializable<raw_status_6_t> rawData;
        rawData.adc1 = htons(adc1) * 1000.0;
        rawData.adc2 = htons(adc2) * 1000.0;
        rawData.adc3 = htons(adc3) * 1000.0;
        rawData.ppm = htons(ppm) * 1000.0;
        return rawData.serializeData();
    }

    VescParameterGroup::VescParameterGroup(uint8_t vescId, steady_clock::duration transmissionInterval) : _vescId(vescId)
    {
        using namespace addressing::drive::vesc;
        _transmissions.emplace_back(
            address_t(vescId, command_id::SET_RPM),
            PacketManager::transmission_config_t{
                .generator = [this]() -> auto
                {
                    return _setRpm.serializeData();
                },
                .interval = transmissionInterval,
                .shouldTransmitImmediately = false,
            });

        _callbacks.emplace_back(
            filter_t{
                .address = static_cast<flagged_address_t>(address_t(vescId, command_id::STATUS_1)),
            },
            PacketManager::callback_config_t{
                .dataCallback = [this](const Packet& packet)
                {
                    _status1.deserializeData(packet.getData());
                },
            });
        _callbacks.emplace_back(
            filter_t{
                .address = static_cast<flagged_address_t>(address_t(vescId, command_id::STATUS_2)),
            },
            PacketManager::callback_config_t{
                .dataCallback = [this](const Packet& packet)
                {
                    _status2.deserializeData(packet.getData());
                },
            });
        _callbacks.emplace_back(
            filter_t{
                .address = static_cast<flagged_address_t>(address_t(vescId, command_id::STATUS_3)),
            },
            PacketManager::callback_config_t{
                .dataCallback = [this](const Packet& packet)
                {
                    _status3.deserializeData(packet.getData());
                },
            });
        _callbacks.emplace_back(
            filter_t{
                .address = static_cast<flagged_address_t>(address_t(vescId, command_id::STATUS_4)),
            },
            PacketManager::callback_config_t{
                .dataCallback = [this](const Packet& packet)
                {
                    _status4.deserializeData(packet.getData());
                },
            });
        _callbacks.emplace_back(
            filter_t{
                .address = static_cast<flagged_address_t>(address_t(vescId, command_id::STATUS_5)),
            },
            PacketManager::callback_config_t{
                .dataCallback = [this](const Packet& packet)
                {
                    _status5.deserializeData(packet.getData());
                },
            });
        _callbacks.emplace_back(
            filter_t{
                .address = static_cast<flagged_address_t>(address_t(vescId, command_id::STATUS_6)),
            },
            PacketManager::callback_config_t{
                .dataCallback = [this](const Packet& packet)
                {
                    _status6.deserializeData(packet.getData());
                },
            });
    }
}

namespace hi_can::parameters::legacy::drive::motors
{
    EscParameterGroup::EscParameterGroup(const addressing::legacy::address_t& deviceAddress)
        : _deviceAddress(deviceAddress)
    {
        using namespace hi_can::addressing::legacy;
        using namespace hi_can::addressing::legacy::drive::motors;

        const auto speedAddress = static_cast<flagged_address_t>(
            address_t(deviceAddress,
                      static_cast<uint8_t>(mcb::groups::ESC),
                      static_cast<uint8_t>(esc::parameter::SPEED)));

        _transmissions.emplace_back(
            speedAddress,
            PacketManager::transmission_config_t{
                .generator = [this]() -> auto
                {
                    return _speed.serializeData();
                },
                .interval = 100ms,
                .shouldTransmitImmediately = false,
            });

        _callbacks.emplace_back(
            filter_t{
                .address = speedAddress,
            },
            PacketManager::callback_config_t{
                .dataCallback = [this](const Packet& packet)
                {
                    _speed.deserializeData(packet.getData());
                },
            });
        _callbacks.emplace_back(
            filter_t{
                .address = static_cast<flagged_address_t>(
                    address_t(deviceAddress,
                              static_cast<uint8_t>(mcb::groups::ESC),
                              static_cast<uint8_t>(esc::parameter::STATUS))),
            },
            PacketManager::callback_config_t{
                .dataCallback = [this](const Packet& packet)
                {
                    _status.deserializeData(packet.getData());
                },
            });
        _callbacks.emplace_back(
            filter_t{
                .address = static_cast<flagged_address_t>(
                    address_t(deviceAddress,
                              static_cast<uint8_t>(mcb::groups::ESC),
                              static_cast<uint8_t>(esc::parameter::POSITION))),
            },
            PacketManager::callback_config_t{
                .dataCallback = [this](const Packet& packet)
                {
                    _position = SimpleSerializable<wrapped_value_t<int64_t>>(packet.getData()).value;
                },
            });
        _callbacks.emplace_back(
            filter_t{
                .address = static_cast<flagged_address_t>(
                    address_t(deviceAddress,
                              static_cast<uint8_t>(mcb::groups::ESC),
                              static_cast<uint8_t>(esc::parameter::LIMITS))),
            },
            PacketManager::callback_config_t{
                .dataCallback = [this](const Packet& packet)
                {
                    if (!_limits)
                        _limits = limits_t();
                    _limits->deserializeData(packet.getData());
                },
            });
    }
    EscParameterGroup::EscParameterGroup(const EscParameterGroup& group)
        : EscParameterGroup(group._deviceAddress)
    {
        _speed = group._speed;
        _status = group._status;
    }

    std::vector<Packet> EscParameterGroup::getStartupTransmissions() const
    {
        using namespace hi_can::addressing::legacy;
        using namespace hi_can::addressing::legacy::drive::motors;

        std::vector<Packet> packets;
        packets.emplace_back(
            flagged_address_t{
                address_t(_deviceAddress,
                          static_cast<uint8_t>(mcb::groups::ESC),
                          static_cast<uint8_t>(esc::parameter::LIMITS)),
                true,
            });
        return packets;
    }
}