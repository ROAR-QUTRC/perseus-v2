#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <chrono>
#include <memory>

class CmdVelToOdomNode : public rclcpp::Node
{
public:
    CmdVelToOdomNode()
    : Node("cmd_vel_to_odom_node")
    {
        // Initialize position and orientation
        x_ = 0.0;
        y_ = 0.0;
        theta_ = 0.0;
        last_time_ = this->get_clock()->now();

        // Create subscriber
        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&CmdVelToOdomNode::cmd_vel_callback, this, std::placeholders::_1));

        // Create publisher
        publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

        // Create timer (10 Hz)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&CmdVelToOdomNode::publish_odom, this));
    }

private:
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // Get current time
        auto current_time = this->get_clock()->now();
        
        // Calculate time difference
        double dt = (current_time - last_time_).seconds();
        
        // Extract velocities
        double vx = msg->linear.x;
        double vy = msg->linear.y;
        double vth = msg->angular.z;
        
        // Calculate change in position and orientation
        double delta_x = (vx * cos(theta_) - vy * sin(theta_)) * dt;
        double delta_y = (vx * sin(theta_) + vy * cos(theta_)) * dt;
        double delta_th = vth * dt;
        
        // Update position and orientation
        x_ += delta_x;
        y_ += delta_y;
        theta_ += delta_th;
        
        // Normalize theta to [-pi, pi]
        theta_ = atan2(sin(theta_), cos(theta_));
        
        last_time_ = current_time;
    }

    void publish_odom()
    {
        auto odom_msg = nav_msgs::msg::Odometry();
        
        // Header
        auto current_time = this->get_clock()->now();
        odom_msg.header.stamp = current_time;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";
        
        // Position
        odom_msg.pose.pose.position.x = x_;
        odom_msg.pose.pose.position.y = y_;
        odom_msg.pose.pose.position.z = 0.0;
        
        // Orientation
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, theta_);
        odom_msg.pose.pose.orientation = tf2::toMsg(q);
        
        // Publish
        publisher_->publish(odom_msg);
    }

    // Member variables
    double x_, y_, theta_;
    rclcpp::Time last_time_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CmdVelToOdomNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}