#include "bucket_driver/bucket_driver.hpp"

#include <algorithm>
#include <cstdint>
#include <hi_can_address.hpp>

BucketDriver::BucketDriver(const rclcpp::NodeOptions& options)
    : Node("bucket_driver", options)
{
    _canInterface = hi_can::RawCanInterface(this->declare_parameter("can_bus", "can0"));
    _killTimer = this->create_timer(ACTUATOR_TIMEOUT, std::bind(&BucketDriver::_timeoutCallback, this));
    _actuatorSubscription =
        this->create_subscription<actuator_msgs::msg::Actuators>("bucket_actuators", 10, std::bind(&BucketDriver::_actuatorCallback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Bucket driver node initialized");
}

void BucketDriver::_actuatorCallback(const actuator_msgs::msg::Actuators::SharedPtr msg)
{
    constexpr size_t ACTUATOR_COUNT = 4;
    if ((msg->velocity.size() != ACTUATOR_COUNT) || (msg->normalized.size() != 1))
    {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Received actuator message with incorrect number of actuators");
        return;
    }

    _killTimer->reset();
    _timedOut = false;
    _writeActuators(msg->velocity[0], msg->velocity[1], msg->velocity[2], msg->velocity[3], std::abs(msg->normalized[0]) > 0.5);
}
void BucketDriver::_timeoutCallback()
{
    if (!_timedOut)
    {
        _writeActuators(0, 0, 0, 0, false);
        _timedOut = true;
    }
}

void BucketDriver::_writeActuators(double lift, double tilt, double jaws, double rotate, bool magnet)
{
    constexpr double ACTUATOR_MAX_SPEED = 0.1;  // m/s
    constexpr double ROTATE_MAX_SPEED = 1.5;    // rad/s (this is a rough guesstimate)
    constexpr double LINEAR_CONVERSION_FACTOR = INT16_MAX / ACTUATOR_MAX_SPEED;
    constexpr double ANGULAR_CONVERSION_FACTOR = INT16_MAX / ROTATE_MAX_SPEED;

    // actuator speeds -> PWM values
    const int16_t liftPwm = static_cast<int16_t>(
        std::clamp(lift * LINEAR_CONVERSION_FACTOR,
                   static_cast<double>(INT16_MIN),
                   static_cast<double>(INT16_MAX)));
    const int16_t tiltPwm = static_cast<int16_t>(
        std::clamp(tilt * LINEAR_CONVERSION_FACTOR,
                   static_cast<double>(INT16_MIN),
                   static_cast<double>(INT16_MAX)));
    const int16_t jawsPwm = static_cast<int16_t>(
        std::clamp(jaws * LINEAR_CONVERSION_FACTOR,
                   static_cast<double>(INT16_MIN),
                   static_cast<double>(INT16_MAX)));
    const int16_t rotatePwm = static_cast<int16_t>(
        std::clamp(rotate * ANGULAR_CONVERSION_FACTOR,
                   static_cast<double>(INT16_MIN),
                   static_cast<double>(INT16_MAX)));

    using namespace hi_can;
    using namespace addressing;
    using namespace addressing::excavation;
    using namespace addressing::excavation::bucket;
    using namespace parameters::excavation::bucket::controller;

    const standard_address_t baseAddress{SYSTEM_ID,
                                         bucket::SUBSYSTEM_ID,
                                         bucket::controller::DEVICE_ID};

    _canInterface.transmit(
        Packet(static_cast<addressing::flagged_address_t>(
                   standard_address_t{baseAddress,
                                      static_cast<uint8_t>(controller::group::LIFT_BOTH),
                                      static_cast<uint8_t>(controller::actuator_parameter::SPEED)}),
               speed_t{liftPwm}.serializeData()));

    _canInterface.transmit(
        Packet(static_cast<addressing::flagged_address_t>(
                   standard_address_t{baseAddress,
                                      static_cast<uint8_t>(controller::group::TILT_BOTH),
                                      static_cast<uint8_t>(controller::actuator_parameter::SPEED)}),
               speed_t{tiltPwm}.serializeData()));

    _canInterface.transmit(
        Packet(static_cast<addressing::flagged_address_t>(
                   standard_address_t{baseAddress,
                                      static_cast<uint8_t>(controller::group::JAWS_BOTH),
                                      static_cast<uint8_t>(controller::actuator_parameter::SPEED)}),
               speed_t{jawsPwm}.serializeData()));

    _canInterface.transmit(
        Packet(static_cast<addressing::flagged_address_t>(
                   standard_address_t{baseAddress,
                                      static_cast<uint8_t>(controller::group::MAGNET),
                                      static_cast<uint8_t>(controller::magnet_parameter::ROTATE_SPEED)}),
               speed_t{rotatePwm}.serializeData()));

    _canInterface.transmit(
        Packet(static_cast<addressing::flagged_address_t>(
                   standard_address_t{baseAddress,
                                      static_cast<uint8_t>(controller::group::MAGNET),
                                      static_cast<uint8_t>(controller::magnet_parameter::MAGNET_ENABLE)}),
               magnet_t{magnet}.serializeData()));
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    try
    {
        auto node = std::make_shared<BucketDriver>();
        RCLCPP_INFO(rclcpp::get_logger("main"), "Starting bucket driver node");
        rclcpp::spin(node);
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Error running bucket driver: %s", e.what());
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
