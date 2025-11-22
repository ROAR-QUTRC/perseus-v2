#pragma once

#include <chrono>
#include <hi_can_twai.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

using namespace hi_can;
using namespace addressing::legacy;
using namespace addressing::legacy::power::control;
using namespace parameters::legacy::power::control::power_bus;

const address_t rcbAddress{power::SYSTEM_ID,
                           power::control::SUBSYSTEM_ID,
                           static_cast<uint8_t>(device::ROVER_CONTROL_BOARD)};

PowerBusParameterGroup computeBus{rcbAddress, power::control::rcb::groups::COMPUTE_BUS};
PowerBusParameterGroup driveBus{rcbAddress, power::control::rcb::groups::DRIVE_BUS};
PowerBusParameterGroup auxBus{rcbAddress, power::control::rcb::groups::AUX_BUS};
PowerBusParameterGroup spareBus{rcbAddress, power::control::rcb::groups::SPARE_BUS};

std::optional<PacketManager> packetManager;

class SystemStatus : public rclcpp::Node
{
public:
    explicit SystemStatus(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    void _publishSystemStatusCallBack(const system_status_msgs::msg::SystemStatus::SharedPtr msg)

        constexpr std::chrono::duration callBackPeriod_ms = 100ms;

    rclcpp::Publisher<system_status_msgs::msg::SystemStatus>::SharedPtr _systemStatusPublisher;
    rclcpp::TimerBase::SharedPtr _callBackTimer;
}