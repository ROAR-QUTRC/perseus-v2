#include "rcb_driver/rcb_driver.hpp"

#include <algorithm>
#include <cstdint>
#include <iostream>
#include <nlohmann/json.hpp>

RcbDriver::RcbDriver(const rclcpp::NodeOptions& options)
    : Node("rcb_driver", options)
{
    using namespace hi_can;
    using namespace addressing::legacy;
    using namespace addressing::legacy::power::control::rcb;
    using namespace parameters::legacy::power::control::power_bus;

    try
    {
        _can_interface.emplace(RawCanInterface(this->declare_parameter("can_bus", "can0")));
        _packet_manager.emplace(_can_interface.value());

        _packet_manager->setCallback(addressing::filter_t(power::SYSTEM_ID, power::control::SUBSYSTEM_ID,
                                                          static_cast<uint8_t>(power::control::device::ROVER_CONTROL_BOARD)),
                                     {.dataCallback =
                                          [this](const hi_can::Packet& packet)
                                      {
                                          this->_can_to_ros(packet);
                                      }});
    }
    catch (const std::exception& e)
    {
        RCLCPP_FATAL(this->get_logger(), "Failed to initialise CAN bus: %s", e.what());
        return;
    }

    _packet_publisher = this->create_publisher<std_msgs::msg::String>("can_to_ros", 10);
    _packet_timeout_timer = this->create_wall_timer(PACKET_TIMEOUT, std::bind(&RcbDriver::_call_receive, this));
    _packet_subscriber = this->create_subscription<std_msgs::msg::String>(
        "ros_to_can", 10, std::bind(&RcbDriver::_ros_to_can, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Rover control board driver node initialized");
}

void RcbDriver::_call_receive()
{
    try
    {
        _packet_manager->handleReceive();
    }
    catch (const std::exception& e)
    {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Receive failed: %s", e.what());
    }
}

void RcbDriver::_can_to_ros(const hi_can::Packet& packet)
{
    using namespace hi_can;
    using namespace addressing::legacy::power;
    using namespace addressing::legacy::power::control;

    // Find the bus group and process accordingly
    for (const auto& [name, id] : this->BUS_GROUPS)
    {
        const auto target_address = addressing::legacy::address_t(
            SYSTEM_ID,
            SUBSYSTEM_ID,
            static_cast<uint8_t>(device::ROVER_CONTROL_BOARD),
            static_cast<uint8_t>(id),
            static_cast<uint8_t>(power_bus::parameter::POWER_STATUS));
        if (packet.getAddress() == static_cast<int>(target_address))
        {
            const auto& raw_data = packet.getData();
            std::string dataStr;
            for (std::size_t i = 0; i < raw_data.size(); i++)
            {
                dataStr += std::format(" {:02X}", static_cast<unsigned int>(raw_data.at(i)));
            }

            parameters::legacy::power::control::power_bus::status_t data;
            data.deserializeData(raw_data);

            // RCLCPP_INFO(this->get_logger(), "message: %s\t#%s -> current: %05d, voltage: %05d, status: %d", name.c_str(), dataStr.c_str(), data.current, data.voltage, data.status);

            auto message = std_msgs::msg::String();
            nlohmann::json bus_data = {{"name", name}, {"current", data.current}, {"voltage", data.voltage}, {"status", static_cast<int>(data.status)}};

            message.data = bus_data.dump();

            this->_packet_publisher->publish(message);
            return;
        }
    }
}

void RcbDriver::_ros_to_can(std_msgs::msg::String::UniquePtr msg)
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

        _can_interface->transmit(Packet(static_cast<addressing::flagged_address_t>(address),
                                        immediate_control_t(_immediate_control_t{data["on"].get<std::string>()[0] == '1', data["clear"].get<std::string>()[0] == '1', 0}).serializeData()));
    }
    catch (const std::exception& e)
    {
        RCLCPP_WARN(get_logger(), "Error transmitting packet: %s", e.what());
    }
}

void RcbDriver::cleanup()
{
    _packet_manager.reset();
    _can_interface.reset();
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