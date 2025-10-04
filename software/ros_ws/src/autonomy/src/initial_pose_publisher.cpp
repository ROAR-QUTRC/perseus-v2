#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class InitialPosePublisher : public rclcpp::Node
{
public:
    InitialPosePublisher()
        : Node("initial_pose_publisher")
    {
        // Create publisher for initial pose
        pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 10);

        // Create a timer to publish the initial pose after a short delay
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(2000),  // Wait 2 seconds
            std::bind(&InitialPosePublisher::publish_initial_pose, this));

        RCLCPP_INFO(this->get_logger(), "Initial pose publisher node started");
    }

private:
    void publish_initial_pose()
    {
        auto pose_msg = geometry_msgs::msg::PoseWithCovarianceStamped();

        // Set header
        pose_msg.header.stamp = this->now();
        pose_msg.header.frame_id = "map";

        // Set initial position (origin of the map)
        pose_msg.pose.pose.position.x = 0.0;
        pose_msg.pose.pose.position.y = 0.0;
        pose_msg.pose.pose.position.z = 0.0;

        // Set initial orientation (facing forward)
        tf2::Quaternion q;
        q.setRPY(0, 0, 0);  // Roll=0, Pitch=0, Yaw=0 (facing forward)
        pose_msg.pose.pose.orientation.x = q.x();
        pose_msg.pose.pose.orientation.y = q.y();
        pose_msg.pose.pose.orientation.z = q.z();
        pose_msg.pose.pose.orientation.w = q.w();

        // Set covariance (uncertainty)
        pose_msg.pose.covariance[0] = 0.25;                  // x
        pose_msg.pose.covariance[7] = 0.25;                  // y
        pose_msg.pose.covariance[35] = 0.06853891945200942;  // yaw

        // Publish the initial pose
        pose_publisher_->publish(pose_msg);

        // Calculate yaw from quaternion manually
        double yaw = atan2(2.0 * (pose_msg.pose.pose.orientation.w * pose_msg.pose.pose.orientation.z +
                                  pose_msg.pose.pose.orientation.x * pose_msg.pose.pose.orientation.y),
                           1.0 - 2.0 * (pose_msg.pose.pose.orientation.y * pose_msg.pose.pose.orientation.y +
                                        pose_msg.pose.pose.orientation.z * pose_msg.pose.pose.orientation.z));

        RCLCPP_INFO(this->get_logger(), "Published initial pose: (%.2f, %.2f, %.2f)",
                    pose_msg.pose.pose.position.x,
                    pose_msg.pose.pose.position.y,
                    yaw);

        // Stop the timer after publishing once
        timer_->cancel();

        // Shutdown the node after a short delay
        auto shutdown_timer = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            [this]()
            {
                RCLCPP_INFO(this->get_logger(), "Initial pose publisher shutting down");
                rclcpp::shutdown();
            });
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<InitialPosePublisher>());
    rclcpp::shutdown();
    return 0;
}