#pragma once

#include <actuator_msgs/msg/actuators.hpp>
#include <chrono>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <string>

class GenericController : public rclcpp::Node
{
public:
    explicit GenericController(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    friend class AxisParser;
    friend class EnableParser;

    class EnableParser;
    class AxisParser
    {
    public:
        AxisParser(GenericController& parent, const std::string& param_base_name, bool has_enable = false);

        double get_value();

    private:
        std::vector<rclcpp::Parameter> _resolve_params();
        static std::vector<std::string> _get_param_names(const std::string& base_name);
        bool _parent_has_all_params(const std::string& follows_name);
        bool _parent_has_any_params(const std::string& follows_name);

        static constexpr size_t AXIS_IDX = 0;
        static constexpr size_t BUTTON_POSITIVE_IDX = 1;
        static constexpr size_t BUTTON_NEGATIVE_IDX = 2;
        static constexpr size_t SCALING_IDX = 3;
        static constexpr size_t FOLLOWS_IDX = 4;
        static constexpr size_t DEADBAND_POSITIVE_IDX = 5;
        static constexpr size_t DEADBAND_NEGATIVE_IDX = 6;
        static constexpr size_t TURBO_IDX = 7;

        std::map<std::string, std::pair<int, rcl_interfaces::msg::ParameterDescriptor>>
            _int_param_map;
        std::map<std::string, std::pair<double, rcl_interfaces::msg::ParameterDescriptor>>
            _double_param_map;
        std::map<std::string, std::pair<std::string, rcl_interfaces::msg::ParameterDescriptor>>
            _string_param_map;

        GenericController& _parent;
        const std::string _param_base_name;
        bool _has_enable;
        double _last_value = 0.0;
    };

    class EnableParser
    {
    public:
        EnableParser(GenericController& parent, const std::string& param_base_name, bool is_turbo = false);

        bool get_value();

    private:
        std::vector<rclcpp::Parameter> _resolve_params();
        static std::vector<std::string> _get_param_names(const std::string& base_name);

        static constexpr size_t FOLLOWS_IDX = 0;
        static constexpr size_t THRESHOLD_IDX = 1;
        static constexpr size_t IS_LT_IDX = 2;

        GenericController& _parent;
        const std::string _param_base_name;
    };
    void _joy_timeout_callback(void);
    void _joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);

    static constexpr std::string FORWARD_BASE_NAME = "drive.forward";
    static constexpr std::string TURN_BASE_NAME = "drive.turn";
    static constexpr std::string LIFT_BASE_NAME = "bucket.lift";
    static constexpr std::string TILT_BASE_NAME = "bucket.tilt";
    static constexpr std::string JAWS_BASE_NAME = "bucket.jaws";
    static constexpr std::string ROTATE_BASE_NAME = "bucket.rotate";
    static constexpr std::string MAGNET_BASE_NAME = "bucket.magnet";
    static constexpr std::string TIMEOUT_LENGTH = "timeout_ns";

    static constexpr auto JOY_TIMEOUT = std::chrono::milliseconds(100);

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr _joy_subscription;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr _twist_publisher;
    rclcpp::Publisher<actuator_msgs::msg::Actuators>::SharedPtr _actuator_publisher;
    rclcpp::TimerBase::SharedPtr _joy_timeout_timer;

protected:
    sensor_msgs::msg::Joy::SharedPtr _last_received_joy;
    rclcpp::Time _prev_received_joy_time;
    std::map<std::string, AxisParser> _axis_parsers;
    std::map<std::string, EnableParser> _enable_parsers;
};
