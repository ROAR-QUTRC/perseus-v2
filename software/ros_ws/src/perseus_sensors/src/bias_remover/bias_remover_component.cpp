// bias_remover_component.cpp
// ------------------------------------------------------------
// Bias Remover (Composable) - subtracts latest bias from IMU gyro
// Adds: output_rate_hz throttle (publishes corrected IMU at target rate)
//
// Topics:
//   Sub: imu_in_topic (sensor_msgs/Imu) [required]
//   Sub: bias_in_topic (geometry_msgs/Vector3Stamped) [required]
//   Pub: imu_out_topic (sensor_msgs/Imu)
//
// Params:
//   imu_in_topic (string) default: "imu"
//   imu_out_topic (string) default: "imu_bias_corrected"
//   bias_in_topic (string) default: "bias"
//   zero_when_no_bias (bool) default: false
//   output_rate_hz (double) default: 50.0  // <=0 means "no throttling"
//
// ------------------------------------------------------------

#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <string>

namespace imu_processors
{

    class BiasRemover : public rclcpp::Node
    {
    public:
        explicit BiasRemover(const rclcpp::NodeOptions& options)
            : rclcpp::Node("imu_bias_remover", options)
        {
            imu_in_topic_ = declare_parameter<std::string>("imu_in_topic", "imu");
            imu_out_topic_ = declare_parameter<std::string>("imu_out_topic", "imu_bias_corrected");
            bias_in_topic_ = declare_parameter<std::string>("bias_in_topic", "bias");

            zero_when_no_bias_ = declare_parameter<bool>("zero_when_no_bias", false);

            // Rate control (NEW)
            output_rate_hz_ = declare_parameter<double>("output_rate_hz", 50.0);
            if (output_rate_hz_ > 0.0)
            {
                output_min_dt_ = rclcpp::Duration::from_seconds(1.0 / output_rate_hz_);
            }
            else
            {
                output_min_dt_ = rclcpp::Duration(0, 0);
            }
            last_output_pub_ = this->get_clock()->now();

            pub_ = create_publisher<sensor_msgs::msg::Imu>(imu_out_topic_, rclcpp::SensorDataQoS());

            bias_sub_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
                bias_in_topic_, 10,
                std::bind(&BiasRemover::bias_cb, this, std::placeholders::_1));

            imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
                imu_in_topic_, rclcpp::SensorDataQoS(),
                std::bind(&BiasRemover::imu_cb, this, std::placeholders::_1));
        }

    private:
        void bias_cb(const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr& msg)
        {
            latest_bias_ = msg->vector;
            latest_bias_stamp_ = msg->header.stamp;
        }

        void imu_cb(const sensor_msgs::msg::Imu::ConstSharedPtr& msg)
        {
            const auto now = this->get_clock()->now();

            // Throttle publishing rate (NEW)
            if (output_rate_hz_ > 0.0 && (now - last_output_pub_) < output_min_dt_)
            {
                return;
            }
            last_output_pub_ = now;

            sensor_msgs::msg::Imu out(*msg);

            if (!latest_bias_.has_value())
            {
                if (zero_when_no_bias_)
                {
                    out.angular_velocity.x = 0.0;
                    out.angular_velocity.y = 0.0;
                    out.angular_velocity.z = 0.0;
                }
                pub_->publish(out);
                return;
            }

            const auto& b = latest_bias_.value();
            out.angular_velocity.x -= b.x;
            out.angular_velocity.y -= b.y;
            out.angular_velocity.z -= b.z;

            pub_->publish(out);
        }

    private:
        // topics
        std::string imu_in_topic_, imu_out_topic_, bias_in_topic_;

        // behavior
        bool zero_when_no_bias_{false};

        // rate control
        double output_rate_hz_{50.0};
        rclcpp::Duration output_min_dt_{0, 0};
        rclcpp::Time last_output_pub_{0, 0, RCL_ROS_TIME};

        // latest bias
        std::optional<geometry_msgs::msg::Vector3> latest_bias_;
        rclcpp::Time latest_bias_stamp_{0, 0, RCL_ROS_TIME};

        // ROS interfaces
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
        rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr bias_sub_;
    };

}  // namespace imu_processors

RCLCPP_COMPONENTS_REGISTER_NODE(imu_processors::BiasRemover)