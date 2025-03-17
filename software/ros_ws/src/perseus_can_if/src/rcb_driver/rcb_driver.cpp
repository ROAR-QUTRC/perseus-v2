#include "rcb_driver/rcb_driver.hpp"

#include <algorithm>
#include <cstdint>

RcbDriver::RcbDriver(const rclcpp::NodeOptions& options) : Node("rcb_driver", options)
{
    try
    {
        _canInterface.emplace(hi_can::RawCanInterface(this->declare_parameter("can_bus", "can0")));
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
            _parameterGroups.emplace_back(address_t(power::SYSTEM_ID, power::control::SUBSYSTEM_ID,
                                                    static_cast<uint8_t>(power::control::device::ROVER_CONTROL_BOARD)),
                                          group);

            _packetManager->addGroup(_parameterGroups.back());
        }
        catch (const std::exception& e)
        {
            RCLCPP_FATAL(this->get_logger(), "Failed to set up parameter group: %s", e.what());
        }
    }

    // TODO: This should be moved over to a packet manager callback in future. This is currently not possible.
    // init publisher and subscriber
    _packetPublisher = this->create_publisher<std_msgs::msg::String>("can_to_ros", 10);
    _packetTimeoutTimer = this->create_wall_timer(PACKET_TIMEOUT, std::bind(&RcbDriver::_canToRos, this));
    _packetSubscriber = this->create_subscription<std_msgs::msg::String>(
        "ros_to_can", 10, std::bind(&RcbDriver::_rosToCan, this, std::placeholders::_1));

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
        message.data += std::format("\"current\": \"{0}\", \"voltage\": \"{1}\", \"power_off\": \"{2}\"", data.current,
                                    data.voltage, isOff);
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
        // Messages are expected to be a 2 digit string where
        // the first digit is the index of the bus
        // and the second represents a boolean value
        const uint8_t bus = msg->data[0] - '0';
        if (bus >= this->BUS_GROUPS.size())
        {
            RCLCPP_WARN(get_logger(), "Invalid bus group: %d", bus);
            return;
        }
        const bool turnOn = (msg->data[1] - '0') == 1;

        // Log the bus group for debugging purposes
        std::string busName;
        switch (this->BUS_GROUPS[bus])
        {
        case groups::COMPUTE_BUS:
            busName = "compute";
            break;
        case groups::DRIVE_BUS:
            busName = "drive";
            break;
        case groups::AUX_BUS:
            busName = "aux";
            break;
        case groups::SPARE_BUS:
            busName = "spare";
            break;
        default:
            busName = "unknown";
        }
        RCLCPP_INFO(get_logger(), "Telling %s bus to turn %s", busName.c_str(), turnOn ? "on" : "off");

        const address_t address(power::SYSTEM_ID, power::control::SUBSYSTEM_ID,
                                static_cast<uint8_t>(power::control::device::ROVER_CONTROL_BOARD),
                                static_cast<uint8_t>(this->BUS_GROUPS[bus]),
                                static_cast<uint8_t>(power::control::power_bus::parameter::CONTROL_IMMEDIATE));

        _canInterface->transmit(Packet(static_cast<addressing::flagged_address_t>(address),
                                       immediate_control_t(_immediate_control_t{turnOn, false, 0}).serializeData()));
    }
    catch (const std::exception& e)
    {
        RCLCPP_WARN(get_logger(), "Error transmitting packet: %s", e.what());
    }
}

void RcbDriver::cleanup()
{
    _packetManager.reset();
    _canInterface.reset();
    _parameterGroups.clear();
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    try
    {
        auto node = std::make_shared<RcbDriver>();
        RCLCPP_INFO(rclcpp::get_logger("main"), "Starting rcb driver node");
        rclcpp::spin(node);
        node->cleanup();
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Error running rcb driver: %s", e.what());
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
