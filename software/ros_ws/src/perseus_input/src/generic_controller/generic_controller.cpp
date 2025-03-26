#include "generic_controller/generic_controller.hpp"

#include <algorithm>
#include <cstdint>
#include <map>

GenericController::GenericController(const rclcpp::NodeOptions& options)
    : Node("generic_controller", options),
      _forwardParser(*this, "drive.forward"),
      _turnParser(*this, "drive.turn"),
      _liftParser(*this, "bucket.lift"),
      _tiltParser(*this, "bucket.tilt"),
      _jawsParser(*this, "bucket.jaws"),
      _rotateParser(*this, "bucket.rotate")
{
    this->create_subscription<sensor_msgs::msg::Joy>("joy", 10,
                                                     std::bind(&GenericController::_joyCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Generic controller initialized");
}

void GenericController::_joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
}

GenericController::AxisParser::AxisParser(GenericController& parent, const std::string& paramBaseName)
    : _parent(parent),
      _paramBaseName(paramBaseName)
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

    _doubleParamMap = {{"scaling",
                        {1.0, scalingDescriptor}}};

    _parent.declare_parameters(paramBaseName, _intParamMap);
    _parent.declare_parameters(paramBaseName, _doubleParamMap);
}

double GenericController::AxisParser::getValue()
{
    auto params = _parent.get_parameters({_paramBaseName + ".axis",
                                          _paramBaseName + ".button_positive",
                                          _paramBaseName + ".button_negative",
                                          _paramBaseName + ".scaling"});
    double scaling = params[3].as_double();

    double axisValue = 0.0;
    if (int axis = params[0].as_int(); axis >= 0)
        axisValue = _parent._lastReceivedJoy->axes[axis];

    bool buttonPositive = false;
    if (int button = params[1].as_int(); button >= 0)
        buttonPositive = _parent._lastReceivedJoy->buttons[button];

    bool buttonNegative = false;
    if (int button = params[2].as_int(); button >= 0)
        buttonNegative = _parent._lastReceivedJoy->buttons[button];

    bool bothButtons = buttonPositive && buttonNegative;
    if (!bothButtons)
    {
        if (buttonPositive)
            axisValue = scaling;
        if (buttonNegative)
            axisValue = -scaling;
    }

    return axisValue;
}

GenericController::EnableParser::EnableParser(GenericController& parent,
                                              const std::string& paramBaseName)
    : _parent(parent),
      _axisParser(parent, paramBaseName),
      _paramBaseName(paramBaseName)
{
    using namespace rcl_interfaces::msg;

    ParameterDescriptor followsDescriptor{};
    followsDescriptor.type = ParameterType::PARAMETER_STRING;
    followsDescriptor.description = "The enable configuration to base initial parameters off of";
    _parent.declare_parameter(paramBaseName + ".follows", "");
}

bool GenericController::EnableParser::getValue()
{
    return false;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    try
    {
        auto node = std::make_shared<GenericController>();
        RCLCPP_INFO(rclcpp::get_logger("main"), "Starting generic controller node");
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
