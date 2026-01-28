#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "cv_bridge/cv_bridge.hpp"
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/aruco.hpp>

// ---- custom interfaces ----
#include "perseus_interfaces/srv/detect_objects.hpp"
#include "perseus_interfaces/msg/object_detections.hpp"

namespace perseus_vision
{

class ArucoDetector : public rclcpp::Node
{
public:
  using DetectObjects = perseus_interfaces::srv::DetectObjects;

  ArucoDetector();

private:
  // Callbacks
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  void compressedImageCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg);

  // Core
  void processImage(const cv::Mat& frame, const std_msgs::msg::Header& header);
  void transformAndPublishMarker(
    const std_msgs::msg::Header& header,
    int marker_id,
    const cv::Vec3d& rvec,
    const cv::Vec3d& tvec);

  // Helpers
  tf2::Quaternion rotationMatrixToQuaternion(const cv::Mat& rotation_matrix);

  // Service
  void handle_request(
    const std::shared_ptr<DetectObjects::Request> request,
    std::shared_ptr<DetectObjects::Response> response);

  // -------------------------
  // Parameters
  // -------------------------
  double marker_length_{0.35};
  double axis_length_{0.03};
  int dictionary_id{1};

  std::string camera_frame_{"camera_link_optical"};
  std::string tf_output_frame_{"odom"};

  std::string input_img_{"/rgbd_camera/image_raw"};
  std::string output_img_{"/detection/aruco/image"};

  bool publish_tf_{true};
  bool publish_img_{true};
  bool compressed_io_{false};
  bool publish_output_{false};
  std::string output_topic_{"/detection/aruco/detections"};

  // Camera intrinsics
  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;

  // ArUco
  cv::aruco::ArucoDetector detector_;

  // TF
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  // ROS IO
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;

  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_sub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_pub_;

  rclcpp::Service<DetectObjects>::SharedPtr service_;

  rclcpp::Publisher<perseus_interfaces::msg::ObjectDetections>::SharedPtr detection_pub_;

  // Cached detections for service + topic
  std::mutex detections_mutex_;
  rclcpp::Time latest_timestamp_{0, 0, RCL_ROS_TIME};
  std::vector<int> latest_ids_;
  std::vector<geometry_msgs::msg::Pose> latest_poses_;
  bool has_detections_{false};
};

}  // namespace perseus_vision
