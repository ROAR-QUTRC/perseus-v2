#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class OdomTransform : public rclcpp::Node
{
public:
    OdomTransform()
        : Node("odom_transform")
    {
        correction_.setRPY(0.0, 0.0, M_PI / 2.0);

        sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom_raw", 10,
            std::bind(&OdomTransform::callback, this, std::placeholders::_1));

        pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

        RCLCPP_INFO(this->get_logger(), "Odom transform node started, applying 90deg CCW correction");
    }

private:
    void callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        nav_msgs::msg::Odometry out = *msg;

        tf2::Quaternion q_in;
        tf2::fromMsg(msg->pose.pose.orientation, q_in);
        tf2::Quaternion q_out = correction_ * q_in;
        q_out.normalize();
        out.pose.pose.orientation = tf2::toMsg(q_out);

        tf2::Vector3 pos_in(
            msg->pose.pose.position.x,
            msg->pose.pose.position.y,
            msg->pose.pose.position.z);
        tf2::Vector3 pos_out = tf2::quatRotate(correction_, pos_in);
        out.pose.pose.position.x = pos_out.x();
        out.pose.pose.position.y = pos_out.y();
        out.pose.pose.position.z = pos_out.z();

        tf2::Vector3 vel_in(
            msg->twist.twist.linear.x,
            msg->twist.twist.linear.y,
            msg->twist.twist.linear.z);
        tf2::Vector3 vel_out = tf2::quatRotate(correction_, vel_in);
        out.twist.twist.linear.x = vel_out.x();
        out.twist.twist.linear.y = vel_out.y();
        out.twist.twist.linear.z = vel_out.z();

        pub_->publish(out);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
    tf2::Quaternion correction_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomTransform>());
    rclcpp::shutdown();
    return 0;
}