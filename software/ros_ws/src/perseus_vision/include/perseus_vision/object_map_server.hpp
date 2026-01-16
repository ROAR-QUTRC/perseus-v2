#ifndef OBJECT_MAP_SERVER_HPP
#define OBJECT_MAP_SERVER_HPP

#include <mutex>
#include <map>
#include <cmath>
#include <yaml-cpp/yaml.h>
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

class ObjectMapServer : public rclcpp::Node
{
public:
    ObjectMapServer();

private:
    void aruco_callback(
        const perseus_vision::msg::ObjectDetections::SharedPtr msg);
    
    void load_targets_and_broadcast();
    void broadcast_target_frames();

    // ROS interfaces
    // Aruco Markers Subscriptions
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr aruco_image_sub_; // for uncompressed images (aruco markers)
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr aruco_image_compressed_sub_; // for compressed images (aruco markers)
    rclcpp::Subscription<perseus_vision::msg::ObjectDetections>::SharedPtr aruco_detections_sub_; // for object detections (aruco markers)
    int capture_radius_;// radius around detected object to capture image

    // Cube Detection Subscriptions
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr cube_image_sub_; // for uncompressed images (cube detection)
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr cube_image_compressed_sub_; // for compressed images (cube detection)
    rclcpp::Subscription<perseus_vision::msg::ObjectDetections>::SharedPtr cube_detections_sub_; // for object detections (cube detection)
    
    // TF2 Broadcasting
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr tf_broadcast_timer_;
    
    // Targets status storage
    struct Target {
        std::string frame_id;
        double x, y, z;  // position
        double qx, qy, qz, qw;  // quaternion
        bool locked = false;  // if true, target cannot be renamed by ArUco detections
    };
    std::map<std::string, Target> targets_;
    std::string targets_file_;
    
    // Node Parameters
    bool publish_tf_;
    bool capture_aruco_;
    bool capture_cube_;
    std::string tf_output_frame_;
    bool capture_images_;
    std::string aruco_detect_topic_;
    std::string cube_detect_topic_;
};

#endif  // OBJECT_MAP_SERVER_HPP
