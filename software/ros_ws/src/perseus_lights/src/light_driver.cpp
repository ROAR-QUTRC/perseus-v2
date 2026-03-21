#include "perseus_lights/light_driver.hpp"

#include <algorithm>
#include <cstdint>
#include <hi_can_address.hpp>

LightDriver::LightDriver(const rclcpp::NodeOptions& options)
    : Node("light_driver", options)
{
    _can_interface = hi_can::RawCanInterface(this->declare_parameter("can_bus", "can0"));
    _command_subscription = this->create_subscription<std_msgs::msg::Byte>("light_status", 10, std::bind(&LightDriver::_control_callback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Light Status Driver node initialised");
}

void LightDriver::_control_callback(const std_msgs::msg::Byte::SharedPtr msg)
{
    using namespace ring;

    colours colour = colours::RED;

    switch (static_cast<commands>(msg->data))
    {
    case commands::WHITE:
        colour = colours::WHITE;
        break;
    case commands::RED:
        colour = colours::RED;
        break;
    case commands::BLUE:
        colour = colours::BLUE;
        break;
    case commands::CYAN:
        colour = colours::CYAN;
        break;
    case commands::GREEN:
        colour = colours::GREEN;
        break;
    case commands::YELLOW:
        colour = colours::YELLOW;
        break;
    case commands::MAGENTA:
        colour = colours::MAGENTA;
        break;
    default:
        RCLCPP_WARN(this->get_logger(), "Unknown light command: %d", msg->data);
        break;
    }

    _write_lights(colour);
}

void LightDriver::_write_lights(ring::colours colour)
{
    using namespace hi_can;
    using namespace addressing;
    using namespace addressing::status_light;
    using namespace addressing::status_light::control::colour;
    using namespace parameters::status_light::control::colour;

    constexpr standard_address_t LIGHT_ADDRESS{
        status_light::SYSTEM_ID,
        status_light::control::SUBSYSTEM_ID,
        status_light::control::colour::DEVICE,
    };

    _can_interface.transmit(
        Packet(
            static_cast<flagged_address_t>(
                standard_address_t{LIGHT_ADDRESS,
                                   static_cast<uint8_t>(group::RING),
                                   static_cast<uint8_t>(parameter::RGB)}),
            rgba_t{static_cast<uint32_t>(colour)}.serialize_data()));
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    try
    {
        auto node = std::make_shared<LightDriver>();
        RCLCPP_INFO(rclcpp::get_logger("main"), "Starting light driver node");
        rclcpp::spin(node);
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Error running light driver: %s", e.what());
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}