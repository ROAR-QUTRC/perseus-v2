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