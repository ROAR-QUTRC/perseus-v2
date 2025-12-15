#include "system_status.hpp"

using namespace hi_can;
using namespace addressing::legacy;
using namespace addressing::legacy::power::control::rcb;
using namespace parameters::legacy::power::control::power_bus;

SystemStatus::SystemStatus(const rclcpp::NodeOptions& options)
    : Node("system_status", options)
{
    // Initialise the CAN interface
    _canInterface.emplace(hi_can::RawCanInterface(this->declare_parameter("can_bus", "can0")));
    _packetManager.emplace(_canInterface.value());

    _rcbSubscriber = this->create_subscription<std_msgs::msg::String>("can_to_ros", 10, std::bind(&SystemStatus::_rcbCallback, this, std::placeholders::_1));
}

void SystemStatus::_sendColourToTowerOverCan(Colour colour)
{
}

void SystemStatus::_rcbCallback(std_msgs::msg::String::UniquePtr msg)
{
    std::vector<power_status> _subsystem_statuses = {};
    std::vector<power_status> _subsystem_names = {};

    auto _subsystem_data = nlohmann::json::parse(msg->data);
    for (i = 0; i < (int)_subsystem_data.size(); i++)
    {
        auto _subsystem_key = (_subsystem_data.begin() + i).key();
        auto _subsystem_values = (_subsystem_data.begin() + i).value();

        _subsystem_statuses.emplace_back(static_cast<power_status>(_subsystem_values["status"]));
        _subsystem_names.emplace_back(static_cast<std::string>(_subsystem_key));

        RCLCPP_INFO(get_logger(), "ROS %d Status: %d", _subsystem_key, _subsystem_data);
    }

    if (!_subsystem_statuses.empty)
    {
        if (std::find(_subsystem_statuses.begin(), _subsystem_statuses.end(), power_status::SHORT_CIRCUIT | power_status::SWITCH_FAILED | power_status::OVERLOAD | power_status::FAULT) != _subsystem_statuses.end())
        {
            _sendColourToTowerOverCan(Colour::RED);
        }
        else if (std::find(_subsystem_statuses.begin(), _subsystem_statuses.end(), /* FILL FOR MAGENTA */) != _subsystem_statuses.end())
        {
            _sendColourToTowerOverCan(Colour::MAGENTA);
        }
        else if (std::find(_subsystem_statuses.begin(), _subsystem_statuses.end(), /* FILL FOR BLUE */) != _subsystem_statuses.end())
        {
            _sendColourToTowerOverCan(Colour::BLUE);
        }
        else if (std::find(_subsystem_statuses.begin(), _subsystem_statuses.end(), /* FILL FOR CYAN */) != _subsystem_statuses.end())
        {
            _sendColourToTowerOverCan(Colour::CYAN);
        }
        else if (std::find(_subsystem_statuses.begin(), _subsystem_statuses.end(), /* FILL FOR GREEN */) != _subsystem_statuses.end())
        {
            _sendColourToTowerOverCan(Colour::GREEN);
        }
    }

    // White (R+G+B): Interactive – safe interaction with rover is possible, rover is not going to start moving until light changes
    // (ensure new light colour – likely blue or cyan – is visible for a short countdown before starting any motion when transitioning out of this state)

    // Blue: Motion – motion enabled, rover is under manual control
    // Cyan (G+B): Initiating automatic motion – will begin to move automatically soon
    // Green: Automatic motion – currently carrying out an automatic program of movement (ensure a short countdown in the cyan state before transitioning into this one)
    // Yellow (G+R): Locked – mechanically locked or otherwise inoperable
    // Magenta (R+B): Conflict – rover starting up, conflicting light state signals, or error with the indicator light (make this the default/timeout state for the light)
    // Red: Error – an error with the rover beyond the indicator light has occurred

    // switch (status)
    // {
    // case power_status::OFF:
    //     return CRGB::Black;
    // case power_status::ON:
    //     return CRGB::Green;
    // case power_status::PRECHARGING:
    //     return CRGB::Yellow;
    // case power_status::SHORT_CIRCUIT:
    //     return CRGB::Red;
    // case power_status::SWITCH_FAILED:
    //     return blinkOn ? CRGB::OrangeRed : CRGB::Black;
    // case power_status::OVERLOAD:
    //     return CRGB::Blue;
    // case power_status::FAULT:
    //     return blinkOn ? CRGB::Red : CRGB::Black;
    // default:
    //     return CRGB::Black;
    // }

    // enum class power_status : uint8_t
    // {
    //     OFF = 0,        // bus off
    //     ON,             // bus on
    //     PRECHARGING,    // bus is precharging
    //     SHORT_CIRCUIT,  // precharging failed - short circuit
    //     SWITCH_FAILED,  // main switch not turning on - estop?
    //     OVERLOAD,       // software fuse triggered
    //     FAULT,          // switch reporting fault
    // };
}

void SystemStatus::cleanup()
{
    _packetManager.reset();
    _canInterface.reset();
}
