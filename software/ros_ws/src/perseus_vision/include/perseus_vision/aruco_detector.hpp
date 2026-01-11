#ifndef ARUCO_DETECTOR_HPP
#define ARUCO_DETECTOR_HPP

#include <mutex>
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
#include <sensor_msgs/msg/compressed_image.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "builtin_interfaces/msg/time.hpp"
#include "perseus_vision/srv/detect_objects.hpp"
#include "perseus_vision/msg/object_detections.hpp"

using DetectObjects = perseus_vision::srv::DetectObjects;

class ArucoDetector : public rclcpp::Node
{
public:
    ArucoDetector();

private:
    // ROS callbacks and helpers
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void compressedImageCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg);
    void processImage(const cv::Mat& frame, const std_msgs::msg::Header& header);
    void transformAndPublishMarker(const std_msgs::msg::Header& header, int marker_id,
                                   const cv::Vec3d& rvec, const cv::Vec3d& tvec);
    tf2::Quaternion rotationMatrixToQuaternion(const cv::Mat& rotation_matrix);
    void handle_request(const std::shared_ptr<DetectObjects::Request> request,
                       std::shared_ptr<DetectObjects::Response> response);

    // ROS interfaces
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_; // for uncompressed images 
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_sub_; // for compressed images
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_; // for uncompressed images 
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_pub_;// for compressed images 
    rclcpp::Publisher<perseus_vision::msg::ObjectDetections>::SharedPtr detection_pub_;// for publishing detections 
    rclcpp::Service<DetectObjects>::SharedPtr service_; // service for detection requests 
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_; // for publishing TFs
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_; // for TF buffer
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_; // for TF listener

    // ArUco
    cv::aruco::Dictionary dictionary_; 
    cv::aruco::DetectorParameters detector_params_;
    cv::aruco::ArucoDetector detector_;

    // Camera intrinsics
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;

    // Configurable parameters
    int dictionary_id;
    double marker_length_;         // in meters
    double axis_length_;           // for drawing axes
    double min_bounding_box_area_; // minimum bounding box area for filtering
    std::string camera_frame_;     // input frame (e.g. camera_link_optical)
    std::string tf_output_frame_;  // output TF frame (e.g. odom)
    std::string input_img_;      // input image topic
    std::string output_img_;     // output image topic
    bool publish_tf_;      // whether to transform to output frame
    bool publish_img_;     // whether to publish processed image
    bool compressed_io_;  // whether input/output images are compressed
    bool use_sim_time_;   // whether to use simulation time 
    bool publish_output_;
    std::string output_topic_;

    // Latest detections cache for service requests
    mutable std::mutex detections_mutex_;
    std::vector<int> latest_ids_;
    std::vector<geometry_msgs::msg::Pose> latest_poses_;
    builtin_interfaces::msg::Time latest_timestamp_;
    bool has_detections_ = false;
};

#endif  // ARUCO_DETECTOR_HPP
