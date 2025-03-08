#include "rcb_driver/rcb_driver.hpp"

#include <algorithm>
#include <cstdint>

RcbDriver::RcbDriver(const rclcpp::NodeOptions& options)
    : Node("rcb_driver", options)
{
    try
    {
        _canInterface.emplace(hi_can::RawCanInterface(this->declare_parameter("can_bus", "vcan0")));
        _packetManager.emplace(_canInterface.value());
    }
    catch (const std::exception& e)
    {
        RCLCPP_FATAL(this->get_logger(), "Failed to initialise CAN bus: %s", e.what());
        return;
    }

    try
    {
        // change this to reflect the actual number of groups
        _parameterGroups.reserve(1);
    }
    catch (const std::exception& e)
    {
        RCLCPP_FATAL(this->get_logger(), "Failed to reserve memory for parameter groups: %s", e.what());
        return;
    }

    using namespace hi_can;
    using namespace addressing::legacy;
    using namespace addressing::legacy::power::control::rcb;

    // iterate the id's of the rcb groups enum
    for (auto& group : this->BUS_GROUPS)
    {
        try
        {
            // create parameter groups
            _parameterGroups.emplace_back(
                address_t(
                    power::SYSTEM_ID,
                    power::control::SUBSYSTEM_ID,
                    static_cast<uint8_t>(power::control::device::ROVER_CONTROL_BOARD)),
                group);

            RCLCPP_INFO(
                this->get_logger(),
                "Added parameter group with address %X %X %X %X",
                power::SYSTEM_ID,
                power::control::SUBSYSTEM_ID,
                static_cast<uint8_t>(power::control::device::ROVER_CONTROL_BOARD),
                static_cast<uint8_t>(group));

            _packetManager->addGroup(_parameterGroups.back());
        }
        catch (const std::exception& e)
        {
            RCLCPP_FATAL(this->get_logger(), "Failed to set up parameter group: %s", e.what());
        }
    }

    // TODO: This should be moved over to a packet manager callback in future. This is currently not possible.
    // init publisher and subscriber
    _packetPublisher = this->create_publisher<std_msgs::msg::String>("rcb_packets", 10);
    _packetTimeoutTimer = this->create_wall_timer(PACKET_TIMEOUT, std::bind(&RcbDriver::_canToRos, this));
    _packetSubscriber = this->create_subscription<std_msgs::msg::String>("set_rcb_packets", 10, std::bind(&RcbDriver::_rosToCan, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Rover control board driver node initialized");
}

void RcbDriver::_canToRos()
{
    try
    {
        _packetManager->handleReceive();
    }
    catch (const std::exception& e)
    {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Receive failed: %s", e.what());
    }

    auto message = std_msgs::msg::String();

    message.data = "[";
    bool isFirst = true;
    for (auto& group : _parameterGroups)
    {
        message.data += isFirst ? "{" : ",{";
        isFirst = false;
        const auto& data = group.getStatus();
        const bool isOff = data.status == hi_can::parameters::legacy::power::control::power_bus::power_status::OFF;
        message.data += std::format("\"current\": \"{0}\", \"voltage\": \"{1}\", \"power_off\": \"{2}\"", data.current, data.voltage, isOff);
        message.data += "}";
    }
    message.data += "]";

    this->_packetPublisher->publish(message);
}

void RcbDriver::_rosToCan(std_msgs::msg::String::UniquePtr msg)
{
    using namespace hi_can;
    using namespace addressing::legacy;
    using namespace hi_can::parameters::legacy::power::control::power_bus;

    try
    {
        RCLCPP_INFO(get_logger(), "Transmitting packet: %s", msg->data.c_str());

        const address_t baseAddress(
            power::SYSTEM_ID,
            power::control::SUBSYSTEM_ID,
            static_cast<uint8_t>(power::control::device::ROVER_CONTROL_BOARD));

        // const std::string targetBus = msg->data.

        const address_t address(
            baseAddress,
            static_cast<uint8_t>(power::control::rcb::groups::AUX_BUS),
            static_cast<uint8_t>(power::control::power_bus::parameter::CONTROL_IMMEDIATE));

        _canInterface->transmit(Packet(
            static_cast<addressing::flagged_address_t>(address),
            immediate_control_t({true, false, 0}).serializeData()));
    }
    catch (const std::exception& e)
    {
        RCLCPP_WARN(get_logger(), "Error transmitting packet: %s", e.what());
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    try
    {
        auto node = std::make_shared<RcbDriver>();
        RCLCPP_INFO(rclcpp::get_logger("main"), "Starting rcb driver node");
        rclcpp::spin(node);
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Error running rcb driver: %s", e.what());
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
