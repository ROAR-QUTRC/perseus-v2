#ifndef ARUCO_DETECTOR_HPP
#define ARUCO_DETECTOR_HPP

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <cv_bridge/cv_bridge.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class ArucoDetector : public rclcpp::Node
{
public:
    ArucoDetector();

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void transformAndPublishMarker(const std_msgs::msg::Header& header, int marker_id,
                                   const cv::Vec3d& rvec, const cv::Vec3d& tvec);
    tf2::Quaternion rotationMatrixToQuaternion(const cv::Mat& rotation_matrix);

    // ROS interfaces
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

    // ArUco
    cv::aruco::Dictionary dictionary_;
    cv::aruco::ArucoDetector detector_;

    // Camera intrinsics
    cv::Mat camera_matrix_, dist_coeffs_;
};

#endif  // ARUCO_DETECTOR_HPP