#include "perseus_lights/light_controller.hpp"

#include <termios.h>
#include <unistd.h>

#include <iostream>
#include <thread>

namespace
{
    // Mirror of ring::commands from light_driver.hpp
    enum class commands : uint8_t
    {
        WHITE = 0,
        RED = 1,
        BLUE = 2,
        CYAN = 3,
        GREEN = 4,
        YELLOW = 5,
        MAGENTA = 6,
    };
}

LightController::LightController(const rclcpp::NodeOptions& options)
    : Node("light_controller", options)
{
    _command_publisher = this->create_publisher<std_msgs::msg::Int8>("light_status", 10);

    std::cout << "Light Controller - press a key to change colour:\n"
              << "  1  White\n"
              << "  2  Red\n"
              << "  3  Blue\n"
              << "  4  Cyan\n"
              << "  5  Green\n"
              << "  6  Yellow\n"
              << "  7  Magenta\n"
              << "  q  Quit\n";

    _keyboard_thread = std::thread(&LightController::_run_keyboard_loop, this);

    RCLCPP_INFO(this->get_logger(), "Light Controller node initialised");
}

void LightController::_run_keyboard_loop()
{
    // Put terminal in raw (non-canonical, no-echo) mode so each keypress
    // is delivered immediately without waiting for Enter.
    termios old_to{}, new_to{};
    tcgetattr(STDIN_FILENO, &old_to);
    new_to = old_to;
    new_to.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &new_to);

    const std::unordered_map<char, commands> key_map = {
        {'1', commands::WHITE},
        {'2', commands::RED},
        {'3', commands::BLUE},
        {'4', commands::CYAN},
        {'5', commands::GREEN},
        {'6', commands::YELLOW},
        {'7', commands::MAGENTA},
    };

    char c{};
    while (rclcpp::ok() && read(STDIN_FILENO, &c, 1) == 1) {
        if (c == 'q' || c == 'Q') {
            rclcpp::shutdown();
            break;
        }

        auto it = key_map.find(c);
        if (it == key_map.end()) continue;

        auto msg = std_msgs::msg::Int8{};  // was Byte
        msg.data = static_cast<int8_t>(it->second);  // was uint8_t
        _command_publisher->publish(msg);

        RCLCPP_INFO(this->get_logger(), "Published command: %d", msg.data);
    }

    tcsetattr(STDIN_FILENO, TCSANOW, &old_to);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    try
    {
        auto node = std::make_shared<LightController>();
        rclcpp::spin(node);
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Error: %s", e.what());
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}