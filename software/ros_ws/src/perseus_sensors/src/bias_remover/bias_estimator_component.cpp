#include <cmath>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <string>

namespace imu_processors
{
    static inline bool abslt(double v, double the) { return std::abs(v) < the; }
    static inline double ewma(double alpha, double avg, double meas)
    {
        return alpha * meas + (1.0 - alpha) * avg;
    }

    class BiasEstimator : public rclcpp::Node
    {
    public:
        explicit BiasEstimator(const rclcpp::NodeOptions& options)
            : rclcpp::Node("imu_bias_estimator", options)
        {
            // Topics
            imu_in_topic_ = declare_parameter<std::string>("imu_in_topic", "imu");
            bias_out_topic_ = declare_parameter<std::string>("bias_out_topic", "bias");
            cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "cmd_vel");
            odom_topic_ = declare_parameter<std::string>("odom_topic", "odom");

            // Behavior
            use_cmd_vel_ = declare_parameter<bool>("use_cmd_vel", false);
            use_odom_ = declare_parameter<bool>("use_odom", false);
            use_stamped_ = declare_parameter<bool>("use_stamped", false);
            alpha_ = declare_parameter<double>("accumulator_alpha", 0.01);
            cmd_vel_threshold_ = declare_parameter<double>("cmd_vel_threshold", 0.001);
            odom_threshold_ = declare_parameter<double>("odom_threshold", 0.001);

            // Stationary policy: "OR" or "AND"
            stationary_mode_ = declare_parameter<std::string>("stationary_mode", "OR");  // OR|AND

            // Rate control (NEW)
            estimator_rate_hz_ = declare_parameter<double>("estimator_rate_hz", 50.0);
            if (estimator_rate_hz_ > 0.0)
            {
                estimator_min_dt_ = rclcpp::Duration::from_seconds(1.0 / estimator_rate_hz_);
            }
            else
            {
                estimator_min_dt_ = rclcpp::Duration(0, 0);
            }
            last_estimator_tick_ = this->get_clock()->now();

            accumulator_.x = accumulator_.y = accumulator_.z = 0.0;

            bias_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(bias_out_topic_, 10);

            if (use_cmd_vel_)
            {
                if (use_stamped_)
                {
                    cmd_stamped_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
                        cmd_vel_topic_, rclcpp::SystemDefaultsQoS(),
                        std::bind(&BiasEstimator::cmd_vel_stamped_cb, this, std::placeholders::_1));
                }
                else
                {
                    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
                        cmd_vel_topic_, rclcpp::SystemDefaultsQoS(),
                        std::bind(&BiasEstimator::cmd_vel_cb, this, std::placeholders::_1));
                }
            }

            if (use_odom_)
            {
                odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
                    odom_topic_, rclcpp::SystemDefaultsQoS(),
                    std::bind(&BiasEstimator::odom_cb, this, std::placeholders::_1));
            }

            imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
                imu_in_topic_, rclcpp::SensorDataQoS(),
                std::bind(&BiasEstimator::imu_cb, this, std::placeholders::_1));
        }

    private:
        void cmd_vel_cb(const geometry_msgs::msg::Twist::ConstSharedPtr& m)
        {
            twist_is_zero_ =
                abslt(m->linear.x, cmd_vel_threshold_) && abslt(m->linear.y, cmd_vel_threshold_) &&
                abslt(m->linear.z, cmd_vel_threshold_) && abslt(m->angular.x, cmd_vel_threshold_) &&
                abslt(m->angular.y, cmd_vel_threshold_) && abslt(m->angular.z, cmd_vel_threshold_);
        }

        void cmd_vel_stamped_cb(const geometry_msgs::msg::TwistStamped::ConstSharedPtr& m)
        {
            const auto& t = m->twist;
            twist_is_zero_ =
                abslt(t.linear.x, cmd_vel_threshold_) && abslt(t.linear.y, cmd_vel_threshold_) &&
                abslt(t.linear.z, cmd_vel_threshold_) && abslt(t.angular.x, cmd_vel_threshold_) &&
                abslt(t.angular.y, cmd_vel_threshold_) && abslt(t.angular.z, cmd_vel_threshold_);
        }

        void odom_cb(const nav_msgs::msg::Odometry::ConstSharedPtr& m)
        {
            const auto& t = m->twist.twist;
            odom_is_zero_ =
                abslt(t.linear.x, odom_threshold_) && abslt(t.linear.y, odom_threshold_) &&
                abslt(t.linear.z, odom_threshold_) && abslt(t.angular.x, odom_threshold_) &&
                abslt(t.angular.y, odom_threshold_) && abslt(t.angular.z, odom_threshold_);
        }

        bool stationary() const
        {
            // If a source is disabled, treat it as "ok" for stationary gating.
            const bool cmd_ok = (!use_cmd_vel_) ? true : twist_is_zero_;
            const bool odom_ok = (!use_odom_) ? true : odom_is_zero_;

            if (stationary_mode_ == "AND")
                return cmd_ok && odom_ok;
            return cmd_ok || odom_ok;  // default OR
        }

        void imu_cb(const sensor_msgs::msg::Imu::ConstSharedPtr& msg)
        {
            const auto now = this->get_clock()->now();

            // Throttle estimator work + bias publishing (NEW)
            if (estimator_rate_hz_ > 0.0 && (now - last_estimator_tick_) < estimator_min_dt_)
            {
                return;
            }
            last_estimator_tick_ = now;

            if (stationary())
            {
                accumulator_.x = ewma(alpha_, accumulator_.x, msg->angular_velocity.x);
                accumulator_.y = ewma(alpha_, accumulator_.y, msg->angular_velocity.y);
                accumulator_.z = ewma(alpha_, accumulator_.z, msg->angular_velocity.z);
            }

            geometry_msgs::msg::Vector3Stamped bias;
            bias.header = msg->header;
            bias.vector = accumulator_;
            bias_pub_->publish(bias);
        }

    private:
        // flags
        bool twist_is_zero_{false};
        bool odom_is_zero_{false};

        // params
        bool use_cmd_vel_{false};
        bool use_odom_{false};
        bool use_stamped_{false};
        double cmd_vel_threshold_{0.001};
        double odom_threshold_{0.001};
        double alpha_{0.01};
        std::string stationary_mode_;

        // rate control
        double estimator_rate_hz_{50.0};
        rclcpp::Duration estimator_min_dt_{0, 0};
        rclcpp::Time last_estimator_tick_{0, 0, RCL_ROS_TIME};

        // topics
        std::string imu_in_topic_, bias_out_topic_, cmd_vel_topic_, odom_topic_;

        // bias accumulator
        geometry_msgs::msg::Vector3 accumulator_;

        // ROS interfaces
        rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr bias_pub_;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
        rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_stamped_sub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    };

}  // namespace imu_processors

RCLCPP_COMPONENTS_REGISTER_NODE(imu_processors::BiasEstimator)