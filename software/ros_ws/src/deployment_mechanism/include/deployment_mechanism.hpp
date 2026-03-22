#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <gpiod.hpp>
#include <string>
#include <fstream>
#include <chrono>

namespace deployment_mechanism
{

/// Minimal sysfs PWM driver
class SysfsPWM
{
public:
    SysfsPWM(int chip, int channel, int frequency_hz);
    ~SysfsPWM();

    void set_duty_cycle(double duty);  // 0.0 to 1.0
    void stop();

private:
    std::string path_;
    long period_ns_;

    void write(const std::string & file, const std::string & value);
};

/* Deployment mechanism motor controller node for the Perseus rover.

Subscribes to /deployment_mechanism/motor/cmd (std_msgs/Float32, -1.0 to +1.0)
and drives the motor via the MC33926 motor driver carrier.

Pin assignments (BCM):
  IN1 -> GPIO 17  direction
  IN2 -> GPIO 27  direction
  D2  -> GPIO 18  hardware PWM (PWM0, pwmchip2 on Pi 5)
  EN  -> GPIO 22  enable

Wire SLEW pin HIGH for 20kHz operation.
Enable PWM in /boot/config.txt: dtoverlay=pwm,pin=18,func=2 */

class DeploymentMechanismNode : public rclcpp::Node
{
public:
    DeploymentMechanismNode();
    ~DeploymentMechanismNode();

private:
    // GPIO pin numbers (BCM)
    static constexpr int PIN_IN1 = 17;
    static constexpr int PIN_IN2 = 27;
    static constexpr int PIN_EN  = 22;

    // PWM config — GPIO 18 = PWM0 on Pi 5 (pwmchip2)
    static constexpr int PWM_CHIP      = 2;
    static constexpr int PWM_CHANNEL   = 0;
    static constexpr int PWM_FREQUENCY = 20000;  // 20 kHz ultrasonic

    static constexpr char GPIO_CHIP[] = "/dev/gpiochip4";  // Pi 5

    void cmd_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void watchdog_callback();
    void set_speed(double speed);

    // GPIO
    gpiod::chip gpio_chip_;
    gpiod::line line_in1_;
    gpiod::line line_in2_;
    gpiod::line line_en_;

    // PWM
    std::unique_ptr<SysfsPWM> pwm_;

    // ROS
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr cmd_sub_;
    rclcpp::TimerBase::SharedPtr watchdog_timer_;
    rclcpp::Time last_cmd_time_;
    double cmd_timeout_s_;
};

}  // namespace deployment_mechanism