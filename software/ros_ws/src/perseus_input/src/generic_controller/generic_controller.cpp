#include "generic_controller/generic_controller.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <stdexcept>

GenericController::GenericController(const rclcpp::NodeOptions& options)
    : Node("generic_controller", options)
{
    _axis_parsers.emplace(std::make_pair(FORWARD_BASE_NAME, AxisParser(*this, FORWARD_BASE_NAME, true)));
    _axis_parsers.emplace(std::make_pair(TURN_BASE_NAME, AxisParser(*this, TURN_BASE_NAME, true)));
    _axis_parsers.emplace(std::make_pair(LIFT_BASE_NAME, AxisParser(*this, LIFT_BASE_NAME)));
    _axis_parsers.emplace(std::make_pair(TILT_BASE_NAME, AxisParser(*this, TILT_BASE_NAME)));
    _axis_parsers.emplace(std::make_pair(JAWS_BASE_NAME, AxisParser(*this, JAWS_BASE_NAME)));
    _axis_parsers.emplace(std::make_pair(ROTATE_BASE_NAME, AxisParser(*this, ROTATE_BASE_NAME)));
    _axis_parsers.emplace(std::make_pair(MAGNET_BASE_NAME, AxisParser(*this, "bucket.magnet")));
    _joy_subscription =
        this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&GenericController::_joy_callback, this, std::placeholders::_1));
    _twist_publisher = this->create_publisher<geometry_msgs::msg::TwistStamped>("joy_vel", 10);
    _actuator_publisher = this->create_publisher<actuator_msgs::msg::Actuators>("bucket_actuators", 10);
    if (this->declare_parameter("timeout_enable", true))
    {
        _joy_timeout_timer = this->create_wall_timer(JOY_TIMEOUT, std::bind(&GenericController::_joy_timeout_callback, this));
    }
    _prev_received_joy_time = this->now();
    RCLCPP_INFO(this->get_logger(), "Generic controller initialized");
}

void GenericController::_joy_timeout_callback(void)
{
    static const auto timeout_length = rcl_time_point_value_t(this->declare_parameter(TIMEOUT_LENGTH, 150000000));
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Timeout length is %ld ns", timeout_length);
    if ((this->now().nanoseconds() - _prev_received_joy_time.nanoseconds()) > timeout_length)
    {
        geometry_msgs::msg::TwistStamped twist_msg;
        twist_msg.twist.linear.x = 0;
        twist_msg.twist.angular.z = 0;

        twist_msg.header.stamp = this->now();
        // Publish twist message
        _twist_publisher->publish(twist_msg);

        actuator_msgs::msg::Actuators actuator_msg;
        actuator_msg.velocity.push_back(0);
        actuator_msg.velocity.push_back(0);
        actuator_msg.velocity.push_back(0);
        actuator_msg.velocity.push_back(0);

        // note: inverted, so magnet is released when the button's pressed
        actuator_msg.normalized.push_back(0);

        actuator_msg.header.stamp = this->now();
        // Publish actuator message
        _actuator_publisher->publish(actuator_msg);

        // Warn user if controller is disconnected
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Joy timeout, publishing zero to twist and actuators to stop movement safely");
    }
}

void GenericController::_joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    _last_received_joy = msg;
    _prev_received_joy_time = this->now();
    double forward = _axis_parsers.at(FORWARD_BASE_NAME).get_value();
    double turn = _axis_parsers.at(TURN_BASE_NAME).get_value();
    double lift = _axis_parsers.at(LIFT_BASE_NAME).get_value();
    double tilt = _axis_parsers.at(TILT_BASE_NAME).get_value();
    double jaws = _axis_parsers.at(JAWS_BASE_NAME).get_value();
    double rotate = _axis_parsers.at(ROTATE_BASE_NAME).get_value();
    bool magnet = _axis_parsers.at(MAGNET_BASE_NAME).get_value() > 0.5;

    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Forward: %+2.2f, Turn: %+2.2f, Lift: %+2.2f, Tilt: %+2.2f, Jaws: %+2.2f, Rotate: %+2.2f, Magnet: %d",
                          forward, turn, lift, tilt, jaws, rotate, magnet);

    geometry_msgs::msg::TwistStamped twist_msg;
    twist_msg.twist.linear.x = forward;
    twist_msg.twist.angular.z = turn;

    twist_msg.header.stamp = this->now();
    // Publish twist message
    _twist_publisher->publish(twist_msg);

    actuator_msgs::msg::Actuators actuator_msg;
    actuator_msg.velocity.push_back(lift);
    actuator_msg.velocity.push_back(tilt);
    actuator_msg.velocity.push_back(jaws);
    actuator_msg.velocity.push_back(rotate);

    // note: inverted, so magnet is released when the button's pressed
    actuator_msg.normalized.push_back(magnet);

    actuator_msg.header.stamp = this->now();
    // Publish actuator message
    _actuator_publisher->publish(actuator_msg);
}

GenericController::AxisParser::AxisParser(GenericController& parent, const std::string& param_base_name, bool has_enable)
    : _parent(parent),
      _param_base_name(param_base_name),
      _has_enable(has_enable)
{
    using namespace rcl_interfaces::msg;

    ParameterDescriptor axis_descriptor{};
    axis_descriptor.type = ParameterType::PARAMETER_INTEGER;
    axis_descriptor.description = "The analog axis to use for parsing";

    ParameterDescriptor button_positive_descriptor{};
    button_positive_descriptor.type = ParameterType::PARAMETER_INTEGER;
    button_positive_descriptor.description = "The button to use which will set the axis to its maximum";

    ParameterDescriptor button_negative_descriptor{};
    button_negative_descriptor.type = ParameterType::PARAMETER_INTEGER;
    button_negative_descriptor.description = "The button to use which will set the axis to its minimum";

    _int_param_map = {
        {"axis",
         {-1, axis_descriptor}},
        {"button_positive",
         {-1, button_positive_descriptor}},
        {"button_negative",
         {-1, button_negative_descriptor}},
    };

    ParameterDescriptor scaling_descriptor{};
    scaling_descriptor.type = ParameterType::PARAMETER_DOUBLE;
    scaling_descriptor.description = "The axis output scale";

    ParameterDescriptor deadband_descriptor{};
    deadband_descriptor.type = ParameterType::PARAMETER_DOUBLE;
    deadband_descriptor.description = "The deadband range for the axis. If positive is NaN, no deadband is applied, and if negative is NaN, it default to negative positive deadband";

    ParameterDescriptor turbo_descriptor{};
    turbo_descriptor.type = ParameterType::PARAMETER_DOUBLE;
    turbo_descriptor.description = "Turbo scaling value";

    _double_param_map = {{"scaling",
                          {std::numeric_limits<double>::quiet_NaN(), scaling_descriptor}},
                         {"turbo",
                          {std::numeric_limits<double>::quiet_NaN(), turbo_descriptor}},
                         {"deadband_positive",
                          {0.08, deadband_descriptor}},
                         {"deadband_negative",
                          {std::numeric_limits<double>::quiet_NaN(), deadband_descriptor}}};

    ParameterDescriptor follows_descriptor{};
    follows_descriptor.type = ParameterType::PARAMETER_STRING;
    follows_descriptor.description = "The enable configuration to base initial parameters off of";
    _string_param_map = {{"follows",
                          {"", follows_descriptor}}};

    _parent.declare_parameters(param_base_name, _int_param_map);
    _parent.declare_parameters(param_base_name, _double_param_map);
    _parent.declare_parameters(param_base_name, _string_param_map);

    ParameterDescriptor hold_descriptor{};
    hold_descriptor.type = ParameterType::PARAMETER_BOOL;
    hold_descriptor.description = "Whether or not to hold the previous value if no input received";
    _parent.declare_parameter(param_base_name + ".hold", false, hold_descriptor);

    if (has_enable)
    {
        std::string enable_name = param_base_name + ".enable";
        std::string turbo_name = param_base_name + ".turbo_enable";
        _parent._enable_parsers.emplace(std::make_pair(enable_name, EnableParser(_parent, enable_name)));
        _parent._enable_parsers.emplace(std::make_pair(turbo_name, EnableParser(_parent, turbo_name)));
    }
}

double GenericController::AxisParser::get_value()
{
    if (_has_enable &&
        !_parent._enable_parsers.at(_param_base_name + ".enable").get_value() &&
        !_parent._enable_parsers.at(_param_base_name + ".turbo_enable").get_value())
        return 0.0;

    auto params = _resolve_params();

    double axis_value = 0.0;
    int axis_idx = params[AXIS_IDX].as_int();
    bool has_analog_axis = axis_idx >= 0;
    bool hold = _parent.get_parameter(_param_base_name + ".hold").as_bool();
    if (has_analog_axis)
        axis_value = _parent._last_received_joy->axes[axis_idx];

    double scaling = params[SCALING_IDX].as_double();
    if (_has_enable && _parent._enable_parsers.at(_param_base_name + ".turbo_enable").get_value())
    {
        scaling = params[TURBO_IDX].as_double();
        if (!std::isfinite(scaling))
            scaling = 2.0;
    }
    if (!std::isfinite(scaling))
        scaling = 1.0;

    double deadband_positive = params[DEADBAND_POSITIVE_IDX].as_double();
    double deadband_negative = params[DEADBAND_NEGATIVE_IDX].as_double();
    if (!std::isfinite(deadband_negative))
        deadband_negative = -deadband_positive;

    if (std::isfinite(deadband_positive))
    {
        bool clamped = axis_value < deadband_positive && axis_value > deadband_negative;
        if (clamped)
            axis_value = 0.0;
        else if (axis_value > deadband_positive)
            axis_value -= deadband_positive;
        else if (axis_value < deadband_negative)
            axis_value -= deadband_negative;

        // re-normalize the axis value
        if (!clamped)
        {
            if (axis_value > 0.0)
                axis_value /= (1.0 - std::abs(deadband_positive));
            else
                axis_value /= (1.0 - std::abs(deadband_negative));
        }
    }

    bool button_positive = false;
    if (int button = params[BUTTON_POSITIVE_IDX].as_int(); button >= 0)
        button_positive = _parent._last_received_joy->buttons[button];

    bool button_negative = false;
    if (int button = params[BUTTON_NEGATIVE_IDX].as_int(); button >= 0)
        button_negative = _parent._last_received_joy->buttons[button];

    bool both_buttons = button_positive && button_negative;
    if (!both_buttons)
    {
        if (button_positive)
            axis_value = 1.0;
        if (button_negative)
            axis_value = -1.0;
    }
    else
        axis_value = 0.0;

    axis_value *= scaling;
    bool should_update_analog_axis = has_analog_axis && (!hold || (abs(axis_value) > 0));
    if (button_positive || button_negative || should_update_analog_axis)
        _last_value = axis_value;
    else if (hold)
        return _last_value;

    return axis_value;
}

std::vector<rclcpp::Parameter> GenericController::AxisParser::_resolve_params()
{
    // TODO: Memoize this with invalidation on parameter changes
    auto params = _parent.get_parameters(_get_param_names(_param_base_name));
    // has an axis parser with that key
    if (auto followed = _parent._axis_parsers.find(params[FOLLOWS_IDX].as_string());
        followed != _parent._axis_parsers.end())
    {
        auto followed_params = followed->second._resolve_params();
        if (auto axis = followed_params[AXIS_IDX]; axis.as_int() >= 0)
            params[AXIS_IDX] = axis;
        if (auto button_positive = followed_params[BUTTON_POSITIVE_IDX]; button_positive.as_int() >= 0)
            params[BUTTON_POSITIVE_IDX] = button_positive;
        if (auto button_negative = followed_params[BUTTON_NEGATIVE_IDX]; button_negative.as_int() >= 0)
            params[BUTTON_NEGATIVE_IDX] = button_negative;
        if (auto scaling = followed_params[SCALING_IDX]; std::isfinite(scaling.as_double()))
            params[SCALING_IDX] = scaling;
        if (auto deadband_positive = followed_params[DEADBAND_POSITIVE_IDX]; std::isfinite(deadband_positive.as_double()))
            params[DEADBAND_POSITIVE_IDX] = deadband_positive;
        if (auto deadband_negative = followed_params[DEADBAND_NEGATIVE_IDX]; std::isfinite(deadband_negative.as_double()))
            params[DEADBAND_NEGATIVE_IDX] = deadband_negative;
        if (auto turbo = followed_params[TURBO_IDX]; std::isfinite(turbo.as_double()))
            params[TURBO_IDX] = turbo;
    }

    return params;
}

std::vector<std::string> GenericController::AxisParser::_get_param_names(const std::string& base_name)
{
    return {base_name + ".axis",
            base_name + ".button_positive",
            base_name + ".button_negative",
            base_name + ".scaling",
            base_name + ".follows",
            base_name + ".deadband_positive",
            base_name + ".deadband_negative",
            base_name + ".turbo"};
}

bool GenericController::AxisParser::_parent_has_all_params(const std::string& follows_name)
{
    for (const auto& param : _get_param_names(follows_name))
        if (!_parent.has_parameter(param))
            return false;

    return true;
}

bool GenericController::AxisParser::_parent_has_any_params(const std::string& follows_name)
{
    for (const auto& param : _get_param_names(follows_name))
    {
        if (_parent.has_parameter(param))
            return true;
    }

    return false;
}

GenericController::EnableParser::EnableParser(GenericController& parent,
                                              const std::string& param_base_name,
                                              bool is_turbo)
    : _parent(parent),
      _param_base_name(param_base_name)
{
    (void)is_turbo;
    using namespace rcl_interfaces::msg;

    auto [vals, was_inserted] = _parent._axis_parsers.emplace(
        std::make_pair(param_base_name, AxisParser(parent, param_base_name)));
    if (!was_inserted)
    {
        throw std::runtime_error("Failed to insert axis parser");
    }

    ParameterDescriptor threshold_descriptor{};
    threshold_descriptor.type = ParameterType::PARAMETER_DOUBLE;
    threshold_descriptor.description = "The threshold to compare against";
    _parent.declare_parameter(param_base_name + ".threshold", std::numeric_limits<double>::quiet_NaN(), threshold_descriptor);

    ParameterDescriptor is_less_than_descriptor{};
    is_less_than_descriptor.type = ParameterType::PARAMETER_BOOL;
    is_less_than_descriptor.description = "Whether or not to enable when the axis value is less than the threshold";
    _parent.declare_parameter(param_base_name + ".is_less_than", false, threshold_descriptor);

    // if (is_turbo)
    // {
    //     ParameterDescriptor turbo_descriptor{};
    //     turbo_descriptor.type = ParameterType::PARAMETER_DOUBLE;
    //     turbo_descriptor.description = "";
    //     _parent.declare_parameter(param_base_name + ".turbo_speed", 2.0, turbo_descriptor);
    // }
}

std::vector<rclcpp::Parameter> GenericController::EnableParser::_resolve_params()
{
    // TODO: Memoize this with invalidation on parameter changes
    auto params = _parent.get_parameters(_get_param_names(_param_base_name));
    // has an axis parser with that key
    if (auto followed = _parent._enable_parsers.find(params[FOLLOWS_IDX].as_string());
        followed != _parent._enable_parsers.end())
    {
        auto followed_params = followed->second._resolve_params();
        if (auto threshold = followed_params[THRESHOLD_IDX]; std::isfinite(threshold.as_double()))
        {
            params[THRESHOLD_IDX] = threshold;
            params[IS_LT_IDX] = followed_params[IS_LT_IDX];
        }
    }

    return params;
}

std::vector<std::string> GenericController::EnableParser::_get_param_names(const std::string& base_name)
{
    return {base_name + ".follows",
            base_name + ".threshold",
            base_name + ".is_less_than"};
}

bool GenericController::EnableParser::get_value()
{
    auto params = _resolve_params();

    double threshold = params[THRESHOLD_IDX].as_double();
    if (!std::isfinite(threshold))
        threshold = 0.5;

    bool is_less_than = params[IS_LT_IDX].as_bool();
    double axis_value = _parent._axis_parsers.at(_param_base_name).get_value();
    if (is_less_than)
        return axis_value < threshold;
    return axis_value > threshold;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions node_options{};
    try
    {
        node_options.allow_undeclared_parameters(true);
        RCLCPP_INFO(rclcpp::get_logger("main"), "Starting generic controller node");
        auto node = std::make_shared<GenericController>(node_options);
        rclcpp::spin(node);
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Error running generic controller: %s", e.what());
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
