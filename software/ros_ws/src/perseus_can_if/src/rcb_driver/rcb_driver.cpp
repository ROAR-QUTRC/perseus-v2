#include "rcb_driver/rcb_driver.hpp"

#include <algorithm>
#include <cstdint>
#include <nlohmann/json.hpp>

RcbDriver::RcbDriver(const rclcpp::NodeOptions& options)
    : Node("rcb_driver", options)
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
        _parameterGroups.reserve(4);
    }
    catch (const std::exception& e)
    {
        RCLCPP_FATAL(this->get_logger(), "Failed to reserve memory for parameter groups: %s", e.what());
        return;
    }

    using namespace hi_can;
    using namespace addressing::legacy;
    using namespace addressing::legacy::power::control::rcb;
    using namespace parameters::legacy::power::control::power_bus;

    // iterate the id's of the rcb groups enum
    for (const auto& [name, id] : this->BUS_GROUPS)
    {
        try
        {
            // create parameter groups
            _parameterGroups.emplace_back(std::make_pair(name,
                                                         PowerBusParameterGroup{
                                                             address_t(power::SYSTEM_ID,
                                                                       power::control::SUBSYSTEM_ID,
                                                                       static_cast<uint8_t>(power::control::device::ROVER_CONTROL_BOARD)),
                                                             id}));

            _packetManager->addGroup(_parameterGroups.back().second);
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
    nlohmann::json busData = {};

    for (auto& [name, group] : _parameterGroups)
    {
        const auto& data = group.getStatus();

        static int status = -1;
        if (status != static_cast<int>(data.status))
        {
            status = static_cast<int>(data.status);
            RCLCPP_INFO(get_logger(), "Bus %s status: %d", name.c_str(), status);
        }

        // RCLCPP_INFO(get_logger(), "Publishing message: current: %d, voltage: %d, status: %d", data.current, data.voltage, data.status == hi_can::parameters::legacy::power::control::power_bus::power_status::OFF);
        busData[name] = {{"current", data.current}, {"voltage", data.voltage}, {"status", static_cast<int>(data.status)}};
    }

    message.data = busData.dump();

    this->_packetPublisher->publish(message);
}

void RcbDriver::_rosToCan(std_msgs::msg::String::UniquePtr msg)
{
    using namespace hi_can;
    using namespace addressing::legacy;
    using namespace hi_can::parameters::legacy::power::control::power_bus;

    try
    {
        // Parse the message
        auto data = nlohmann::json::parse(msg->data);
        auto group = std::find_if(BUS_GROUPS.begin(), BUS_GROUPS.end(), [&data](const auto& pair)
                                  { return pair.first == data["bus"].get<std::string>(); });

        RCLCPP_INFO(get_logger(), "Setting power state of bus: %s to %s", data["bus"].get<std::string>().c_str(), data["on"].get<std::string>().c_str());

        using namespace hi_can::addressing::legacy::power::control::rcb;

        const address_t address(power::SYSTEM_ID, power::control::SUBSYSTEM_ID,
                                static_cast<uint8_t>(power::control::device::ROVER_CONTROL_BOARD),
                                static_cast<uint8_t>(group->second),
                                static_cast<uint8_t>(power::control::power_bus::parameter::CONTROL_IMMEDIATE));

        _canInterface->transmit(Packet(static_cast<addressing::flagged_address_t>(address),
                                       immediate_control_t(_immediate_control_t{data["on"].get<std::string>()[0] == '0', false, 0}).serializeData()));
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
