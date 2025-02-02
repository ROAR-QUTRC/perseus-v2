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
        virtual const callback_configs_t& getCallbacks() const { return _callbacks; }
        /// @brief Get the @ref PacketManager::transmission_config_t "transmissions" the group manages
        /// @return Group transmission configs
        virtual const transmission_configs_t& getTransmissions() const { return _transmissions; }
        /// @brief Get a list of @ref Packet "Packets" to send upon adding the group
        /// @return Packets to transmit
        virtual std::vector<Packet> getStartupTransmissions() const { return {}; }

    protected:
        callback_configs_t _callbacks{};
        transmission_configs_t _transmissions{};
    };
    class Serializable
    {
    public:
        virtual std::vector<uint8_t> serializeData() = 0;
    };
    class Deserializable
    {
    public:
        virtual void deserializeData(const std::vector<uint8_t>& serializedData) = 0;
    };
    class BidirectionalSerializable : public Serializable, public Deserializable
    {
    };
    template <double scalingFactor>
    struct scaled_int32_t : public BidirectionalSerializable
    {
        double value = 0;

        void deserializeData(const std::vector<uint8_t>& serializedData) override
        {
            int32_t rawData;
            if (serializedData.size() != sizeof(rawData))
                throw std::invalid_argument("Data size does not match");
            std::copy(serializedData.begin(), serializedData.end(), reinterpret_cast<uint8_t* const>(&rawData));
            value = ntohl(rawData) / scalingFactor;
        }
        std::vector<uint8_t> serializeData() override
        {
            int32_t rawData = htonl(static_cast<int32_t>(round(value * scalingFactor)));
            std::vector<uint8_t> dataBuf;
            dataBuf.resize(sizeof(rawData));
            std::copy_n(reinterpret_cast<uint8_t* const>(&rawData), sizeof(rawData), dataBuf.begin());
            return dataBuf;
        }
    };
    template <double scalingFactor>
    struct scaled_int16_t : public BidirectionalSerializable
    {
        double value = 0;

        void deserializeData(const std::vector<uint8_t>& serializedData) override
        {
            int16_t rawData;
            if (serializedData.size() != sizeof(rawData))
                throw std::invalid_argument("Data size does not match");
            std::copy(serializedData.begin(), serializedData.end(), reinterpret_cast<uint8_t* const>(&rawData));
            value = ntohs(rawData) / scalingFactor;
        }
        std::vector<uint8_t> serializeData() override
        {
            int16_t rawData = htons(static_cast<int16_t>(round(value * scalingFactor)));
            std::vector<uint8_t> dataBuf;
            dataBuf.resize(sizeof(rawData));
            std::copy_n(reinterpret_cast<uint8_t* const>(&rawData), sizeof(rawData), dataBuf.begin());
            return dataBuf;
        }
    };
    template <typename T>
    struct wrapped_value_t
    {
        wrapped_value_t() = default;
        wrapped_value_t(T value) : value(value) {}
        T value{};
    };
    template <typename T>
    class SimpleSerializable : public BidirectionalSerializable, public T
    {
    public:
        using T::T;

        SimpleSerializable(const std::vector<uint8_t>& serializedData)
        {
            deserializeData(serializedData);
        }
        void deserializeData(const std::vector<uint8_t>& serializedData) override
        {
            if (serializedData.size() != sizeof(T))
                throw std::invalid_argument("Data size does not match");
            std::copy(serializedData.begin(), serializedData.end(), reinterpret_cast<uint8_t* const>(static_cast<T*>(this)));
        }
        std::vector<uint8_t> serializeData() override
        {
            std::vector<uint8_t> dataBuf;
            dataBuf.resize(sizeof(T));
            std::copy_n(reinterpret_cast<uint8_t* const>(static_cast<T*>(this)), sizeof(T), dataBuf.begin());
            return dataBuf;
        }
    };

    namespace drive
    {
        namespace vesc
        {
            // COMMAND STRUCTS
            struct set_duty_t : public scaled_int32_t<100000.0>
            {
            };
            struct set_current_t : public scaled_int32_t<1000.0>
            {
            };
            struct set_brake_t : public scaled_int32_t<1000.0>
            {
            };
            struct set_rpm_t : public scaled_int32_t<1.0>
            {
            };
            struct set_pos_t : public scaled_int32_t<1000000.0>
            {
            };
            struct set_current_rel_t : public scaled_int32_t<100000.0>
            {
            };
            struct set_current_brake_rel_t : public scaled_int32_t<100000.0>
            {
            };
            struct set_current_handbrake_t : public scaled_int32_t<1000.0>
            {
            };
            struct set_current_handbrake_rel_t : public scaled_int32_t<100000.0>
            {
            };
            // STATUS STRUCTS
            struct status_1_t : public BidirectionalSerializable
            {
                double rpm = 0;
                double current = 0;
                double dutyCycle = 0;

                void deserializeData(const std::vector<uint8_t>& serializedData) override;
                std::vector<uint8_t> serializeData() override;
            };
            struct status_2_t : public BidirectionalSerializable
            {
                double ah = 0;
                double ahCharge = 0;

                void deserializeData(const std::vector<uint8_t>& serializedData) override;
                std::vector<uint8_t> serializeData() override;
            };
            struct status_3_t : public BidirectionalSerializable
            {
                double wh = 0;
                double whCharge = 0;

                void deserializeData(const std::vector<uint8_t>& serializedData) override;
                std::vector<uint8_t> serializeData() override;
            };
            struct status_4_t : public BidirectionalSerializable
            {
                double tempFet = 0;
                double tempMotor = 0;
                double currentIn = 0;
                double pidPos = 0;

                void deserializeData(const std::vector<uint8_t>& serializedData) override;
                std::vector<uint8_t> serializeData() override;
            };
            struct status_5_t : public BidirectionalSerializable
            {
                double tachometer = 0;
                double voltsIn = 0;

                void deserializeData(const std::vector<uint8_t>& serializedData) override;
                std::vector<uint8_t> serializeData() override;
            };
            struct status_6_t : public BidirectionalSerializable
            {
                double adc1 = 0;
                double adc2 = 0;
                double adc3 = 0;
                double ppm = 0;

                void deserializeData(const std::vector<uint8_t>& serializedData) override;
                std::vector<uint8_t> serializeData() override;
            };

            class VescParameterGroup : public ParameterGroup
            {
            public:
                VescParameterGroup(uint8_t vescId, std::chrono::steady_clock::duration transmissionInterval = std::chrono::steady_clock::duration::zero());

                auto& getStatus1() { return _status1; }
                auto& getStatus2() { return _status2; }
                auto& getStatus3() { return _status3; }
                auto& getStatus4() { return _status4; }
                auto& getStatus5() { return _status5; }
                auto& getStatus6() { return _status6; }

                auto& getSetRpm() { return _setRpm; }

            private:
                uint8_t _vescId = 0;

                set_rpm_t _setRpm;

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
    namespace excavation
    {

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
                    int16_t realSpeed = 0;
                    int16_t realCurrent = 0;
                };
                struct _limits_t
                {
                    int16_t maxCurrent = 0;
                    int16_t rampSpeed = 0;
                };
#pragma pack(pop)
                typedef SimpleSerializable<_speed_t> speed_t;
                typedef SimpleSerializable<_status_t> status_t;
                typedef SimpleSerializable<_limits_t> limits_t;
                class EscParameterGroup : public ParameterGroup
                {
                public:
                    EscParameterGroup(const addressing::legacy::address_t& deviceAddress);

                    EscParameterGroup(const EscParameterGroup&);

                    std::vector<Packet> getStartupTransmissions() const override;

                    auto& getSpeed() { return _speed; }
                    auto& getStatus() { return _status; }
                    auto& getPosition() { return _position; }
                    auto& getLimits() { return _position; }

                private:
                    addressing::legacy::address_t _deviceAddress;
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

                }
                namespace power_bus
                {
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
                    _motor_speed_t(bool enable, int16_t speed) : enable(enable), speed(speed) {}
                    bool enable = 0;
                    int16_t speed = 0;
                };
#pragma pack(pop)
                typedef SimpleSerializable<_motor_speed_t> motor_speed_t;
            }
        }
    }
}