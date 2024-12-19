#include "hi_can_parameter.hpp"

#define DESERIALIZE_BOILERPLATE(_type, _rawData, _serializedData) \
    _type _rawData;                                               \
    if ((_serializedData).size() != sizeof(_rawData))             \
        throw std::invalid_argument("Data size does not match");  \
    std::copy((_serializedData).begin(), (_serializedData).end(), reinterpret_cast<uint8_t*>(&(_rawData)));

#define SERIALIZE_BOILERPLATE(_type, _rawData, _dataBuf) \
    std::vector<uint8_t> _dataBuf;                       \
    _type _rawData;                                      \
    _dataBuf.resize(sizeof(_rawData));

using namespace hi_can;
using namespace parameters;
using namespace addressing;

using namespace std::chrono;
using namespace std::chrono_literals;

namespace hi_can::parameters::drive::vesc
{
#pragma pack(push, 1)
    struct raw_status_t
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

    void status_t::deserializeData(const std::vector<uint8_t>& serializedData)
    {
        DESERIALIZE_BOILERPLATE(raw_status_t, rawData, serializedData);
        rpm = ntohl(rawData.rpm);
        current = ntohs(rawData.current) / 10.0;
        dutyCycle = ntohs(rawData.dutyCycle) / 1000.0;
    }
    std::vector<uint8_t> status_t::serializeData()
    {
        SERIALIZE_BOILERPLATE(raw_status_t, rawData, dataBuf);
        rawData.rpm = htonl(rpm);
        rawData.current = htons(current) * 10.0;
        rawData.dutyCycle = htons(dutyCycle) * 1000.0;
        std::copy_n(reinterpret_cast<uint8_t*>(&rawData), sizeof(rawData), dataBuf.begin());
        return dataBuf;
    }

    void status_2_t::deserializeData(const std::vector<uint8_t>& serializedData)
    {
        DESERIALIZE_BOILERPLATE(raw_status_2_t, rawData, serializedData);
        ah = ntohl(rawData.ah) / 10000.0;
        ahCharge = ntohl(rawData.ahCharge) / 10000.0;
    }
    std::vector<uint8_t> status_2_t::serializeData()
    {
        SERIALIZE_BOILERPLATE(raw_status_2_t, rawData, dataBuf);
        rawData.ah = htonl(ah) * 10000.0;
        rawData.ahCharge = htonl(ahCharge) * 10000.0;
        std::copy_n(reinterpret_cast<uint8_t*>(&rawData), sizeof(rawData), dataBuf.begin());
        return dataBuf;
    }

    void status_3_t::deserializeData(const std::vector<uint8_t>& serializedData)
    {
        DESERIALIZE_BOILERPLATE(raw_status_3_t, rawData, serializedData);
        wh = ntohl(rawData.wh) / 10000.0;
        whCharge = ntohl(rawData.whCharge) / 10000.0;
    }
    std::vector<uint8_t> status_3_t::serializeData()
    {
        SERIALIZE_BOILERPLATE(raw_status_3_t, rawData, dataBuf);
        rawData.wh = htonl(wh) * 10000.0;
        rawData.whCharge = htonl(whCharge) * 10000.0;
        std::copy_n(reinterpret_cast<uint8_t*>(&rawData), sizeof(rawData), dataBuf.begin());
        return dataBuf;
    }

    void status_4_t::deserializeData(const std::vector<uint8_t>& serializedData)
    {
        DESERIALIZE_BOILERPLATE(raw_status_4_t, rawData, serializedData);
        tempFet = ntohs(rawData.tempFet) / 10.0;
        tempMotor = ntohs(rawData.tempMotor) / 10.0;
        currentIn = ntohs(rawData.currentIn) / 10.0;
        pidPos = ntohs(rawData.pidPos) / 50.0;
    }
    std::vector<uint8_t> status_4_t::serializeData()
    {
        SERIALIZE_BOILERPLATE(raw_status_4_t, rawData, dataBuf);
        rawData.tempFet = htons(tempFet) * 10.0;
        rawData.tempMotor = htons(tempMotor) * 10.0;
        rawData.currentIn = htons(currentIn) * 10.0;
        rawData.pidPos = htons(pidPos) * 50.0;
        std::copy_n(reinterpret_cast<uint8_t*>(&rawData), sizeof(rawData), dataBuf.begin());
        return dataBuf;
    }

    void status_5_t::deserializeData(const std::vector<uint8_t>& serializedData)
    {
        DESERIALIZE_BOILERPLATE(raw_status_5_t, rawData, serializedData);
        tachometer = ntohl(rawData.tachometer) / 6.0;
        voltsIn = ntohs(rawData.voltsIn) / 10.0;
    }
    std::vector<uint8_t> status_5_t::serializeData()
    {
        SERIALIZE_BOILERPLATE(raw_status_5_t, rawData, dataBuf);
        rawData.tachometer = htonl(tachometer) * 6.0;
        rawData.voltsIn = htons(voltsIn) * 10.0;
        std::copy_n(reinterpret_cast<uint8_t*>(&rawData), sizeof(rawData), dataBuf.begin());
        return dataBuf;
    }

    void status_6_t::deserializeData(const std::vector<uint8_t>& serializedData)
    {
        DESERIALIZE_BOILERPLATE(raw_status_6_t, rawData, serializedData);
        adc1 = ntohs(rawData.adc1) / 1000.0;
        adc2 = ntohs(rawData.adc2) / 1000.0;
        adc3 = ntohs(rawData.adc3) / 1000.0;
        ppm = ntohs(rawData.ppm) / 1000.0;
    }
    std::vector<uint8_t> status_6_t::serializeData()
    {
        SERIALIZE_BOILERPLATE(raw_status_6_t, rawData, dataBuf);
        rawData.adc1 = htons(adc1) * 1000.0;
        rawData.adc2 = htons(adc2) * 1000.0;
        rawData.adc3 = htons(adc3) * 1000.0;
        rawData.ppm = htons(ppm) * 1000.0;
        std::copy_n(reinterpret_cast<uint8_t*>(&rawData), sizeof(rawData), dataBuf.begin());
        return dataBuf;
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
                .address = static_cast<flagged_address_t>(address_t(vescId, command_id::STATUS)),
            },
            PacketManager::callback_config_t{
                .dataCallback = [this](const Packet& packet)
                {
                    _status.deserializeData(packet.getData());
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
#pragma pack(push, 1)
    struct raw_speed_t
    {
        bool enabled = false;
        motor_direction direction = motor_direction::STOP;
        int16_t speed = 0;
    };
    struct raw_status_t
    {
        bool ready = false;
        int16_t realSpeed = 0;
        int16_t realCurrent = 0;
    };
#pragma pack(pop)

    void speed_t::deserializeData(const std::vector<uint8_t>& serializedData)
    {
        DESERIALIZE_BOILERPLATE(raw_speed_t, rawData, serializedData);
        enabled = rawData.enabled;
        direction = rawData.direction;
        speed = (rawData.speed);
    }
    std::vector<uint8_t> speed_t::serializeData()
    {
        SERIALIZE_BOILERPLATE(raw_speed_t, rawData, dataBuf);
        rawData.enabled = enabled;
        rawData.direction = direction;
        rawData.speed = (speed);
        std::copy_n(reinterpret_cast<uint8_t*>(&rawData), sizeof(rawData), dataBuf.begin());
        return dataBuf;
    }
    void status_t::deserializeData(const std::vector<uint8_t>& serializedData)
    {
        DESERIALIZE_BOILERPLATE(raw_status_t, rawData, serializedData);
        ready = rawData.ready;
        realSpeed = (rawData.realSpeed);
        realCurrent = (rawData.realCurrent);
    }
    std::vector<uint8_t> status_t::serializeData()
    {
        SERIALIZE_BOILERPLATE(raw_status_t, rawData, dataBuf);
        rawData.ready = ready;
        rawData.realSpeed = (realSpeed);
        rawData.realCurrent = (realCurrent);
        std::copy_n(reinterpret_cast<uint8_t*>(&rawData), sizeof(rawData), dataBuf.begin());
        return dataBuf;
    }

    EscParameterGroup::EscParameterGroup(const addressing::legacy::address_t& deviceAddress)
        : _deviceAddress(deviceAddress)
    {
        using namespace hi_can::addressing::legacy;
        using namespace hi_can::addressing::legacy::drive::motors;
        const auto speedAddress = static_cast<flagged_address_t>(
            address_t(deviceAddress,
                      static_cast<uint8_t>(mcb::groups::ESC),
                      static_cast<uint8_t>(esc::parameter::SPEED)));
        const auto statusAddress = static_cast<flagged_address_t>(
            address_t(deviceAddress,
                      static_cast<uint8_t>(mcb::groups::ESC),
                      static_cast<uint8_t>(esc::parameter::STATUS)));

        _transmissions.emplace_back(
            speedAddress,
            PacketManager::transmission_config_t{
                .generator = [this]() -> auto
                {
                    return _speed.serializeData();
                },
                .interval = 100ms,
                .shouldTransmitImmediately = true,
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
                .address = statusAddress,
            },
            PacketManager::callback_config_t{
                .dataCallback = [this](const Packet& packet)
                {
                    _status.deserializeData(packet.getData());
                },
            });
    }
    EscParameterGroup::EscParameterGroup(const EscParameterGroup& group)
        : EscParameterGroup(group._deviceAddress)
    {
        _speed = group._speed;
        _status = group._status;
    }
}