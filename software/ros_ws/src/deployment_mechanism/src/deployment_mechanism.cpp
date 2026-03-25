#include "deployment_mechanism.hpp"

#include <cmath>
#include <filesystem>
#include <stdexcept>

namespace deployment_mechanism
{

    SysfsPWM::SysfsPWM(int chip, int channel, int frequency_hz)
    {
        path_ = "/sys/class/pwm/pwmchip" + std::to_string(chip) +
                "/pwm" + std::to_string(channel);
        period_ns_ = static_cast<long>(1e9 / frequency_hz);

        // Export the channel if not already exported
        if (!std::filesystem::exists(path_))
        {
            std::string export_path = "/sys/class/pwm/pwmchip" +
                                      std::to_string(chip) + "/export";
            write(export_path, std::to_string(channel));
        }

        write(path_ + "/period", std::to_string(period_ns_));
        set_duty_cycle(0.0);
        write(path_ + "/enable", "1");
    }

    SysfsPWM::~SysfsPWM()
    {
        stop();
    }

    void SysfsPWM::set_duty_cycle(double duty)
    {
        duty = std::clamp(duty, 0.0, 1.0);
        long duty_ns = static_cast<long>(duty * period_ns_);
        write(path_ + "/duty_cycle", std::to_string(duty_ns));
    }

    void SysfsPWM::stop()
    {
        set_duty_cycle(0.0);
        write(path_ + "/enable", "0");
    }

    void SysfsPWM::write(const std::string& file, const std::string& value)
    {
        std::ofstream f(file);
        if (!f.is_open())
        {
            throw std::runtime_error("Cannot open sysfs file: " + file);
        }
        f << value;
    }

    DeploymentMechanismNode::DeploymentMechanismNode()
        : Node("deployment_mechanism")
    {
        declare_parameter("cmd_timeout_s", 0.5);
        cmd_timeout_s_ = get_parameter("cmd_timeout_s").as_double();

        // Open GPIO chip
        gpio_chip_ = gpiod::chip(GPIO_CHIP);

        // Request lines
        auto request_output = [&](int pin) -> gpiod::line
        {
            gpiod::line line = gpio_chip_.get_line(pin);
            gpiod::line_request req;
            req.request_type = gpiod::line_request::DIRECTION_OUTPUT;
            req.consumer = "deployment_mechanism";
            line.request(req, 0);  // initial value 0 (inactive)
            return line;
        };

        line_in1_ = request_output(PIN_IN1);
        line_in2_ = request_output(PIN_IN2);
        line_en_ = request_output(PIN_EN);

        // Set up hardware PWM via sysfs
        try
        {
            pwm_ = std::make_unique<SysfsPWM>(PWM_CHIP, PWM_CHANNEL, PWM_FREQUENCY);
        }
        catch (const std::exception& e)
        {
            RCLCPP_FATAL(get_logger(),
                         "Failed to initialise PWM: %s. "
                         "Ensure dtoverlay=pwm,pin=18,func=2 is in /boot/config.txt and reboot.",
                         e.what());
            throw;
        }

        // Enable driver and set speed to 0
        line_en_.set_value(1);
        set_speed(0.0);

        // Subscriber
        cmd_sub_ = create_subscription<std_msgs::msg::Float32>(
            "motor/cmd", 10,
            [this](const std_msgs::msg::Float32::SharedPtr msg)
            {
                cmd_callback(msg);
            });

        // Watchdog timer — 100ms tick
        last_cmd_time_ = now();
        watchdog_timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            [this]()
            { watchdog_callback(); });

        RCLCPP_INFO(get_logger(),
                    "Deployment mechanism motor ready. "
                    "Listening on /deployment_mechanism/motor/cmd — "
                    "PWM @ %dHz, timeout %.1fs",
                    PWM_FREQUENCY, cmd_timeout_s_);
    }

    DeploymentMechanismNode::~DeploymentMechanismNode()
    {
        RCLCPP_INFO(get_logger(), "Shutting down — stopping deployment mechanism motor");
        set_speed(0.0);
        line_en_.set_value(0);
        line_in1_.release();
        line_in2_.release();
        line_en_.release();
    }

    void DeploymentMechanismNode::cmd_callback(
        const std_msgs::msg::Float32::SharedPtr msg)
    {
        double speed = std::clamp(static_cast<double>(msg->data), -1.0, 1.0);
        last_cmd_time_ = now();
        set_speed(speed);
    }

    void DeploymentMechanismNode::watchdog_callback()
    {
        double elapsed = (now() - last_cmd_time_).seconds();
        if (elapsed > cmd_timeout_s_)
        {
            set_speed(0.0);
        }
    }

    void DeploymentMechanismNode::set_speed(double speed)
    {
        if (speed > 0.0)
        {
            line_in1_.set_value(1);
            line_in2_.set_value(0);
            pwm_->set_duty_cycle(speed);
        }
        else if (speed < 0.0)
        {
            line_in1_.set_value(0);
            line_in2_.set_value(1);
            pwm_->set_duty_cycle(std::abs(speed));
        }
        else
        {
            // Brake
            line_in1_.set_value(0);
            line_in2_.set_value(0);
            pwm_->set_duty_cycle(0.0);
        }
    }

}  // namespace deployment_mechanism

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<deployment_mechanism::DeploymentMechanismNode>());
    rclcpp::shutdown();
    return 0;
}