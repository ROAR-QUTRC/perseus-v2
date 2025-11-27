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
    _axisParsers.emplace(std::make_pair(FORWARD_BASE_NAME, AxisParser(*this, FORWARD_BASE_NAME, true)));
    _axisParsers.emplace(std::make_pair(TURN_BASE_NAME, AxisParser(*this, TURN_BASE_NAME, true)));
    _axisParsers.emplace(std::make_pair(LIFT_BASE_NAME, AxisParser(*this, LIFT_BASE_NAME)));
    _axisParsers.emplace(std::make_pair(TILT_BASE_NAME, AxisParser(*this, TILT_BASE_NAME)));
    _axisParsers.emplace(std::make_pair(JAWS_BASE_NAME, AxisParser(*this, JAWS_BASE_NAME)));
    _axisParsers.emplace(std::make_pair(ROTATE_BASE_NAME, AxisParser(*this, ROTATE_BASE_NAME)));
    _axisParsers.emplace(std::make_pair(MAGNET_BASE_NAME, AxisParser(*this, "bucket.magnet")));
    _joySubscription =
        this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&GenericController::_joyCallback, this, std::placeholders::_1));
    _twistPublisher = this->create_publisher<geometry_msgs::msg::TwistStamped>("joy_vel", 10);
    _actuatorPublisher = this->create_publisher<actuator_msgs::msg::Actuators>("bucket_actuators", 10);
    if(this->declare_parameter("timeout_enable", "true") == "true"){
        _joyTimeoutTimer = this->create_wall_timer(JOY_TIMEOUT, std::bind(&GenericController::_joyTimeoutCallback, this));
    }
    _prevReceivedJoyTime = this->now();
    RCLCPP_INFO(this->get_logger(), "Generic controller initialized");
}

void GenericController::_joyTimeoutCallback(void)
{
    static const auto timeout_length = rcl_time_point_value_t(this->declare_parameter(TIMEOUT_LENGTH, 150000000));
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Timeout length is %ld ns", timeout_length);
    if ((this->now().nanoseconds() - _prevReceivedJoyTime.nanoseconds()) > timeout_length)
    {
        geometry_msgs::msg::TwistStamped twistMsg;
        twistMsg.twist.linear.x = 0;
        twistMsg.twist.angular.z = 0;

        twistMsg.header.stamp = this->now();
        // Publish twist message
        _twistPublisher->publish(twistMsg);

        actuator_msgs::msg::Actuators actuatorMsg;
        actuatorMsg.velocity.push_back(0);
        actuatorMsg.velocity.push_back(0);
        actuatorMsg.velocity.push_back(0);
        actuatorMsg.velocity.push_back(0);

        // note: inverted, so magnet is released when the button's pressed
        actuatorMsg.normalized.push_back(0);

        actuatorMsg.header.stamp = this->now();
        // Publish actuator message
        _actuatorPublisher->publish(actuatorMsg);

        // Warn user if controller is disconnected
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Joy timeout, publishing zero to twist and actuators to stop movement safely");
    }
}

void GenericController::_joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    _lastReceivedJoy = msg;
    _prevReceivedJoyTime = this->now();
    double forward = _axisParsers.at(FORWARD_BASE_NAME).getValue();
    double turn = _axisParsers.at(TURN_BASE_NAME).getValue();
    double lift = _axisParsers.at(LIFT_BASE_NAME).getValue();
    double tilt = _axisParsers.at(TILT_BASE_NAME).getValue();
    double jaws = _axisParsers.at(JAWS_BASE_NAME).getValue();
    double rotate = _axisParsers.at(ROTATE_BASE_NAME).getValue();
    bool magnet = _axisParsers.at(MAGNET_BASE_NAME).getValue() > 0.5;

    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Forward: %+2.2f, Turn: %+2.2f, Lift: %+2.2f, Tilt: %+2.2f, Jaws: %+2.2f, Rotate: %+2.2f, Magnet: %d",
                          forward, turn, lift, tilt, jaws, rotate, magnet);

    geometry_msgs::msg::TwistStamped twistMsg;
    twistMsg.twist.linear.x = forward;
    twistMsg.twist.angular.z = turn;

    twistMsg.header.stamp = this->now();
    // Publish twist message
    _twistPublisher->publish(twistMsg);

    actuator_msgs::msg::Actuators actuatorMsg;
    actuatorMsg.velocity.push_back(lift);
    actuatorMsg.velocity.push_back(tilt);
    actuatorMsg.velocity.push_back(jaws);
    actuatorMsg.velocity.push_back(rotate);

    // note: inverted, so magnet is released when the button's pressed
    actuatorMsg.normalized.push_back(magnet);

    actuatorMsg.header.stamp = this->now();
    // Publish actuator message
    _actuatorPublisher->publish(actuatorMsg);
}

GenericController::AxisParser::AxisParser(GenericController& parent, const std::string& paramBaseName, bool hasEnable)
    : _parent(parent),
      _paramBaseName(paramBaseName),
      _hasEnable(hasEnable)
{
    using namespace rcl_interfaces::msg;

    ParameterDescriptor axisDescriptor{};
    axisDescriptor.type = ParameterType::PARAMETER_INTEGER;
    axisDescriptor.description = "The analog axis to use for parsing";

    ParameterDescriptor buttonPositiveDescriptor{};
    buttonPositiveDescriptor.type = ParameterType::PARAMETER_INTEGER;
    buttonPositiveDescriptor.description = "The button to use which will set the axis to its maximum";

    ParameterDescriptor buttonNegativeDescriptor{};
    buttonNegativeDescriptor.type = ParameterType::PARAMETER_INTEGER;
    buttonNegativeDescriptor.description = "The button to use which will set the axis to its minimum";

    _intParamMap = {
        {"axis",
         {-1, axisDescriptor}},
        {"button_positive",
         {-1, buttonPositiveDescriptor}},
        {"button_negative",
         {-1, buttonNegativeDescriptor}},
    };

    ParameterDescriptor scalingDescriptor{};
    scalingDescriptor.type = ParameterType::PARAMETER_DOUBLE;
    scalingDescriptor.description = "The axis output scale";

    ParameterDescriptor deadbandDescriptor{};
    deadbandDescriptor.type = ParameterType::PARAMETER_DOUBLE;
    deadbandDescriptor.description = "The deadband range for the axis. If positive is NaN, no deadband is applied, and if negative is NaN, it default to negative positive deadband";

    ParameterDescriptor turboDescriptor{};
    turboDescriptor.type = ParameterType::PARAMETER_DOUBLE;
    turboDescriptor.description = "Turbo scaling value";

    _doubleParamMap = {{"scaling",
                        {std::numeric_limits<double>::quiet_NaN(), scalingDescriptor}},
                       {"turbo",
                        {std::numeric_limits<double>::quiet_NaN(), turboDescriptor}},
                       {"deadband_positive",
                        {0.08, deadbandDescriptor}},
                       {"deadband_negative",
                        {std::numeric_limits<double>::quiet_NaN(), deadbandDescriptor}}};

    ParameterDescriptor followsDescriptor{};
    followsDescriptor.type = ParameterType::PARAMETER_STRING;
    followsDescriptor.description = "The enable configuration to base initial parameters off of";
    _stringParamMap = {{"follows",
                        {"", followsDescriptor}}};

    _parent.declare_parameters(paramBaseName, _intParamMap);
    _parent.declare_parameters(paramBaseName, _doubleParamMap);
    _parent.declare_parameters(paramBaseName, _stringParamMap);

    ParameterDescriptor holdDescriptor{};
    holdDescriptor.type = ParameterType::PARAMETER_BOOL;
    holdDescriptor.description = "Whether or not to hold the previous value if no input received";
    _parent.declare_parameter(paramBaseName + ".hold", false, holdDescriptor);

    if (hasEnable)
    {
        std::string enableName = paramBaseName + ".enable";
        std::string turboName = paramBaseName + ".turbo_enable";
        _parent._enableParsers.emplace(std::make_pair(enableName, EnableParser(_parent, enableName)));
        _parent._enableParsers.emplace(std::make_pair(turboName, EnableParser(_parent, turboName)));
    }
}

double GenericController::AxisParser::getValue()
{
    if (_hasEnable &&
        !_parent._enableParsers.at(_paramBaseName + ".enable").getValue() &&
        !_parent._enableParsers.at(_paramBaseName + ".turbo_enable").getValue())
        return 0.0;

    auto params = _resolveParams();

    double axisValue = 0.0;
    int axisIdx = params[AXIS_IDX].as_int();
    bool hasAnalogAxis = axisIdx >= 0;
    bool hold = _parent.get_parameter(_paramBaseName + ".hold").as_bool();
    if (hasAnalogAxis)
        axisValue = _parent._lastReceivedJoy->axes[axisIdx];

    double scaling = params[SCALING_IDX].as_double();
    if (_hasEnable && _parent._enableParsers.at(_paramBaseName + ".turbo_enable").getValue())
    {
        scaling = params[TURBO_IDX].as_double();
        if (!std::isfinite(scaling))
            scaling = 2.0;
    }
    if (!std::isfinite(scaling))
        scaling = 1.0;

    double deadbandPositive = params[DEADBAND_POSITIVE_IDX].as_double();
    double deadbandNegative = params[DEADBAND_NEGATIVE_IDX].as_double();
    if (!std::isfinite(deadbandNegative))
        deadbandNegative = -deadbandPositive;

    if (std::isfinite(deadbandPositive))
    {
        bool clamped = axisValue < deadbandPositive && axisValue > deadbandNegative;
        if (clamped)
            axisValue = 0.0;
        else if (axisValue > deadbandPositive)
            axisValue -= deadbandPositive;
        else if (axisValue < deadbandNegative)
            axisValue -= deadbandNegative;

        // re-normalize the axis value
        if (!clamped)
        {
            if (axisValue > 0.0)
                axisValue /= (1.0 - std::abs(deadbandPositive));
            else
                axisValue /= (1.0 - std::abs(deadbandNegative));
        }
    }

    bool buttonPositive = false;
    if (int button = params[BUTTON_POSITIVE_IDX].as_int(); button >= 0)
        buttonPositive = _parent._lastReceivedJoy->buttons[button];

    bool buttonNegative = false;
    if (int button = params[BUTTON_NEGATIVE_IDX].as_int(); button >= 0)
        buttonNegative = _parent._lastReceivedJoy->buttons[button];

    bool bothButtons = buttonPositive && buttonNegative;
    if (!bothButtons)
    {
        if (buttonPositive)
            axisValue = 1.0;
        if (buttonNegative)
            axisValue = -1.0;
    }
    else
        axisValue = 0.0;

    axisValue *= scaling;
    bool shouldUpdateAnalogAxis = hasAnalogAxis && (!hold || (abs(axisValue) > 0));
    if (buttonPositive || buttonNegative || shouldUpdateAnalogAxis)
        _lastValue = axisValue;
    else if (hold)
        return _lastValue;

    return axisValue;
}

std::vector<rclcpp::Parameter> GenericController::AxisParser::_resolveParams()
{
    // TODO: Memoize this with invalidation on parameter changes
    auto params = _parent.get_parameters(_getParamNames(_paramBaseName));
    // has an axis parser with that key
    if (auto followed = _parent._axisParsers.find(params[FOLLOWS_IDX].as_string());
        followed != _parent._axisParsers.end())
    {
        auto followedParams = followed->second._resolveParams();
        if (auto axis = followedParams[AXIS_IDX]; axis.as_int() >= 0)
            params[AXIS_IDX] = axis;
        if (auto buttonPositive = followedParams[BUTTON_POSITIVE_IDX]; buttonPositive.as_int() >= 0)
            params[BUTTON_POSITIVE_IDX] = buttonPositive;
        if (auto buttonNegative = followedParams[BUTTON_NEGATIVE_IDX]; buttonNegative.as_int() >= 0)
            params[BUTTON_NEGATIVE_IDX] = buttonNegative;
        if (auto scaling = followedParams[SCALING_IDX]; std::isfinite(scaling.as_double()))
            params[SCALING_IDX] = scaling;
        if (auto deadbandPositive = followedParams[DEADBAND_POSITIVE_IDX]; std::isfinite(deadbandPositive.as_double()))
            params[DEADBAND_POSITIVE_IDX] = deadbandPositive;
        if (auto deadbandNegative = followedParams[DEADBAND_NEGATIVE_IDX]; std::isfinite(deadbandNegative.as_double()))
            params[DEADBAND_NEGATIVE_IDX] = deadbandNegative;
        if (auto turbo = followedParams[TURBO_IDX]; std::isfinite(turbo.as_double()))
            params[TURBO_IDX] = turbo;
    }

    return params;
}

std::vector<std::string> GenericController::AxisParser::_getParamNames(const std::string& baseName)
{
    return {baseName + ".axis",
            baseName + ".button_positive",
            baseName + ".button_negative",
            baseName + ".scaling",
            baseName + ".follows",
            baseName + ".deadband_positive",
            baseName + ".deadband_negative",
            baseName + ".turbo"};
}

bool GenericController::AxisParser::_parentHasAllParams(const std::string& followsName)
{
    for (const auto& param : _getParamNames(followsName))
        if (!_parent.has_parameter(param))
            return false;

    return true;
}

bool GenericController::AxisParser::_parentHasAnyParams(const std::string& followsName)
{
    for (const auto& param : _getParamNames(followsName))
    {
        if (_parent.has_parameter(param))
            return true;
    }

    return false;
}

GenericController::EnableParser::EnableParser(GenericController& parent,
                                              const std::string& paramBaseName,
                                              bool isTurbo)
    : _parent(parent),
      _paramBaseName(paramBaseName)
{
    (void)isTurbo;
    using namespace rcl_interfaces::msg;

    auto [vals, wasInserted] = _parent._axisParsers.emplace(
        std::make_pair(paramBaseName, AxisParser(parent, paramBaseName)));
    if (!wasInserted)
    {
        throw std::runtime_error("Failed to insert axis parser");
    }

    ParameterDescriptor thresholdDescriptor{};
    thresholdDescriptor.type = ParameterType::PARAMETER_DOUBLE;
    thresholdDescriptor.description = "The threshold to compare against";
    _parent.declare_parameter(paramBaseName + ".threshold", std::numeric_limits<double>::quiet_NaN(), thresholdDescriptor);

    ParameterDescriptor isLessThanDescriptor{};
    isLessThanDescriptor.type = ParameterType::PARAMETER_BOOL;
    isLessThanDescriptor.description = "Whether or not to enable when the axis value is less than the threshold";
    _parent.declare_parameter(paramBaseName + ".is_less_than", false, thresholdDescriptor);

    // if (isTurbo)
    // {
    //     ParameterDescriptor turboDescriptor{};
    //     turboDescriptor.type = ParameterType::PARAMETER_DOUBLE;
    //     turboDescriptor.description = "";
    //     _parent.declare_parameter(paramBaseName + ".turbo_speed", 2.0, turboDescriptor);
    // }
}

std::vector<rclcpp::Parameter> GenericController::EnableParser::_resolveParams()
{
    // TODO: Memoize this with invalidation on parameter changes
    auto params = _parent.get_parameters(_getParamNames(_paramBaseName));
    // has an axis parser with that key
    if (auto followed = _parent._enableParsers.find(params[FOLLOWS_IDX].as_string());
        followed != _parent._enableParsers.end())
    {
        auto followedParams = followed->second._resolveParams();
        if (auto threshold = followedParams[THRESHOLD_IDX]; std::isfinite(threshold.as_double()))
        {
            params[THRESHOLD_IDX] = threshold;
            params[IS_LT_IDX] = followedParams[IS_LT_IDX];
        }
    }

    return params;
}

std::vector<std::string> GenericController::EnableParser::_getParamNames(const std::string& baseName)
{
    return {baseName + ".follows",
            baseName + ".threshold",
            baseName + ".is_less_than"};
}

bool GenericController::EnableParser::getValue()
{
    auto params = _resolveParams();

    double threshold = params[THRESHOLD_IDX].as_double();
    if (!std::isfinite(threshold))
        threshold = 0.5;

    bool isLessThan = params[IS_LT_IDX].as_bool();
    double axisValue = _parent._axisParsers.at(_paramBaseName).getValue();
    if (isLessThan)
        return axisValue < threshold;
    return axisValue > threshold;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions nodeOptions{};
    try
    {
        nodeOptions.allow_undeclared_parameters(true);
        RCLCPP_INFO(rclcpp::get_logger("main"), "Starting generic controller node");
        auto node = std::make_shared<GenericController>(nodeOptions);
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
