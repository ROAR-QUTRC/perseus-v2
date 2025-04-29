#include <Arduino.h>

#include <chrono>
#include <hi_can_twai.hpp>
#include <optional>
#include <thread>

#define IN1 13
#define IN2 14
// #define IN3 21
// #define IN4 47

#define ENA 48
// #define ENB 45

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace hi_can;

std::optional<PacketManager> packetManager;

void handleMotorSpeedData(const Packet& packet);
void setMotorSpeed(int16_t speed);

void setup()
{
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(ENA, OUTPUT);

    // pinMode(IN4, OUTPUT);
    // pinMode(IN3, OUTPUT);
    // pinMode(ENB, OUTPUT);

    auto& interface = TwaiInterface::getInstance();
    packetManager.emplace(interface);
    using namespace addressing;
    using namespace shared::elevator::elevator;
    standard_address_t motorAddress{DEVICE_ADDRESS, motor::GROUP_ID, static_cast<uint8_t>(motor::parameter::SPEED)};
    packetManager->setCallback(filter_t{static_cast<flagged_address_t>(motorAddress)},
                               {
                                   .dataCallback = handleMotorSpeedData,
                                   .timeoutCallback = std::bind(setMotorSpeed, 0),
                                   .timeout = 200ms,
                               });
}

void loop()
{
    using namespace hi_can;
    try
    {
        auto& interface = TwaiInterface::getInstance();
        interface.receiveAll(true);
    }
    catch (const std::exception& e)
    {
        printf(std::format("{}\n", e.what()).c_str());
    }
}

void handleMotorSpeedData(const Packet& packet)
{
    using namespace parameters::shared::elevator::elevator::motor;
    try
    {
        setMotorSpeed(static_cast<int16_t>(std::round(speed_t{packet.getData()}.value)));
    }
    catch (const std::exception& e)
    {
        printf(std::format("Failed to parse speed packet: {}\n", e.what()).c_str());
    }
}

void setMotorSpeed(int16_t speed)
{
    speed = map(speed, std::numeric_limits<int16_t>::min(), std::numeric_limits<int16_t>::max(), -255, 255);
    digitalWrite(IN1, speed > 0);
    digitalWrite(IN2, speed < 0);
    analogWrite(ENA, abs(speed));
}
