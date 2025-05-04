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
        AxisParser(GenericController& parent, const std::string& paramBaseName, bool hasEnable = false);

        double getValue();

    private:
        std::vector<rclcpp::Parameter> _resolveParams();
        static std::vector<std::string> _getParamNames(const std::string& baseName);
        bool _parentHasAllParams(const std::string& followsName);
        bool _parentHasAnyParams(const std::string& followsName);

        static constexpr size_t AXIS_IDX = 0;
        static constexpr size_t BUTTON_POSITIVE_IDX = 1;
        static constexpr size_t BUTTON_NEGATIVE_IDX = 2;
        static constexpr size_t SCALING_IDX = 3;
        static constexpr size_t FOLLOWS_IDX = 4;
        static constexpr size_t DEADBAND_POSITIVE_IDX = 5;
        static constexpr size_t DEADBAND_NEGATIVE_IDX = 6;
        static constexpr size_t TURBO_IDX = 7;

        std::map<std::string, std::pair<int, rcl_interfaces::msg::ParameterDescriptor>>
            _intParamMap;
        std::map<std::string, std::pair<double, rcl_interfaces::msg::ParameterDescriptor>>
            _doubleParamMap;
        std::map<std::string, std::pair<std::string, rcl_interfaces::msg::ParameterDescriptor>>
            _stringParamMap;

        GenericController& _parent;
        const std::string _paramBaseName;
        bool _hasEnable;
        double _lastValue = 0.0;
    };

    class EnableParser
    {
    public:
        EnableParser(GenericController& parent, const std::string& paramBaseName, bool isTurbo = false);

        bool getValue();

    private:
        std::vector<rclcpp::Parameter> _resolveParams();
        static std::vector<std::string> _getParamNames(const std::string& baseName);

        static constexpr size_t FOLLOWS_IDX = 0;
        static constexpr size_t THRESHOLD_IDX = 1;
        static constexpr size_t IS_LT_IDX = 2;

        GenericController& _parent;
        const std::string _paramBaseName;
    };
    void _joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);

    static constexpr std::string FORWARD_BASE_NAME = "drive.forward";
    static constexpr std::string TURN_BASE_NAME = "drive.turn";
    static constexpr std::string LIFT_BASE_NAME = "bucket.lift";
    static constexpr std::string TILT_BASE_NAME = "bucket.tilt";
    static constexpr std::string JAWS_BASE_NAME = "bucket.jaws";
    static constexpr std::string ROTATE_BASE_NAME = "bucket.rotate";
    static constexpr std::string MAGNET_BASE_NAME = "bucket.magnet";

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr _joySubscription;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr _twistPublisher;
    rclcpp::Publisher<actuator_msgs::msg::Actuators>::SharedPtr _actuatorPublisher;

protected:
    sensor_msgs::msg::Joy::SharedPtr _lastReceivedJoy;
    std::map<std::string, AxisParser> _axisParsers;
    std::map<std::string, EnableParser> _enableParsers;
};
