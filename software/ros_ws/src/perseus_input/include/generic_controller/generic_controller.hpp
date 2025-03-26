#pragma once

#include <chrono>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <string>

class GenericController : public rclcpp::Node
{
public:
    explicit GenericController(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

protected:
    sensor_msgs::msg::Joy::SharedPtr _lastReceivedJoy;

private:
    friend class AxisParser;
    friend class EnableParser;
    class AxisParser
    {
    public:
        AxisParser(GenericController& parent, const std::string& paramBaseName);

        double getValue();

    private:
        std::map<std::string, std::pair<int, rcl_interfaces::msg::ParameterDescriptor>>
            _intParamMap;
        std::map<std::string, std::pair<double, rcl_interfaces::msg::ParameterDescriptor>>
            _doubleParamMap;
        GenericController& _parent;
        const std::string _paramBaseName;
    };
    class EnableParser
    {
    public:
        EnableParser(GenericController& parent, const std::string& paramBaseName);

        bool getValue();

    private:
        AxisParser _axisParser;
        GenericController& _parent;
        const std::string _paramBaseName;
    };
    void _joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);

    // drive
    AxisParser _forwardParser;
    AxisParser _turnParser;

    // bucket
    AxisParser _liftParser;
    AxisParser _tiltParser;
    AxisParser _jawsParser;
    AxisParser _rotateParser;

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr _joySubscription;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr _twistPublisher;
};
