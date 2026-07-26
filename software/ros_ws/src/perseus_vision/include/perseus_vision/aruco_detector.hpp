#pragma once

/// @file aruco_detector.hpp
/// @brief ArUco marker detection and 6-DoF pose estimation ROS 2 node.

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <cstdint>
#include <cv_bridge/cv_bridge.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>
#include <mutex>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/region_of_interest.hpp>
#include <std_msgs/msg/header.hpp>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>
#include <visualization_msgs/msg/marker_array.hpp>

#include "perseus_interfaces/msg/object_detections.hpp"
#include "perseus_interfaces/srv/detect_objects.hpp"

namespace perseus_vision
{
    /// @brief A detected marker's ID paired with its position in the camera frame.
    ///
    /// Cached alongside the annotated frame so that captured images can be labelled
    /// with marker coordinates after the detection pass has finished.
    struct marker_coordinate_t
    {
        /// @brief The detected ArUco marker ID.
        int32_t id{0};
        /// @brief Marker position, in metres, in a forward/left/up camera frame.
        cv::Point3d position{};
    };

    /// @brief ROS 2 node for detecting ArUco markers and estimating their 6-DoF poses.
    ///
    /// Subscribes to raw or compressed camera images, detects ArUco markers using OpenCV,
    /// estimates marker poses via cv::solvePnP, and optionally publishes TF transforms,
    /// detection messages, annotated debug images, and rviz visualization markers.
    /// Also provides a service interface for on-demand detection queries and image capture.
    class ArucoDetector : public rclcpp::Node
    {
    public:
        using DetectObjects = perseus_interfaces::srv::DetectObjects;

        /// @brief Constructs the node, declaring parameters and setting up
        ///        subscribers, publishers, and the detection service.
        ArucoDetector();

    private:
        /// @brief Default marker side length, in metres.
        static constexpr double DEFAULT_MARKER_LENGTH_M = 0.35;
        /// @brief Default length of the pose axes drawn onto debug images, in metres.
        static constexpr double DEFAULT_AXIS_LENGTH_M = 0.03;
        /// @brief Default minimum marker bounding box area, in square pixels.
        ///
        /// Detections smaller than this are discarded, as their pose estimates are unreliable.
        static constexpr double DEFAULT_MIN_BOUNDING_BOX_AREA_PX = 100.0;
        /// @brief Default predefined OpenCV ArUco dictionary ID (cv::aruco::DICT_4X4_100).
        static constexpr int32_t DEFAULT_DICTIONARY_ID = 1;

        /// @brief Default frame the camera publishes its images in.
        static inline const std::string DEFAULT_CAMERA_FRAME = "camera_color_optical_frame";
        /// @brief Default frame that marker poses are transformed into before publishing.
        static inline const std::string DEFAULT_TF_OUTPUT_FRAME = "odom";
        /// @brief Default topic that camera images are read from.
        static inline const std::string DEFAULT_INPUT_IMAGE_TOPIC = "/camera/camera/color/image_raw";
        /// @brief Default topic that annotated debug images are published on.
        static inline const std::string DEFAULT_OUTPUT_IMAGE_TOPIC = "/perseus_vision/aruco/image";
        /// @brief Default topic that detection messages are published on.
        static inline const std::string DEFAULT_OUTPUT_DETECTIONS_TOPIC = "/perseus_vision/aruco/detections";
        /// @brief Default topic that camera calibration is read from.
        static inline const std::string DEFAULT_CAMERA_INFO_TOPIC = "/camera/camera/color/camera_info";

        /// @brief Detects markers in a raw image and republishes the annotated result.
        /// @param msg Incoming raw image message.
        void _image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

        /// @brief Detects markers in a compressed image and republishes the annotated result.
        /// @param msg Incoming compressed image message.
        void _compressed_image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg);

        /// @brief Updates the cached camera intrinsics from a camera_info message.
        /// @param msg Incoming camera calibration message.
        void _camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

        /// @brief Runs the full detection pass over one frame and publishes its results.
        /// @param frame Image to detect markers in.
        /// @param header Header of the message the frame came from.
        void _process_image(const cv::Mat& frame, const std_msgs::msg::Header& header);

        /// @brief Discards the previously cached detections and records a new timestamp.
        /// @param stamp Acquisition time of the frame being processed.
        void _reset_latest_detections(const rclcpp::Time& stamp);

        /// @brief Takes a thread-safe copy of the current camera intrinsics.
        /// @param camera_matrix_out Receives the 3x3 intrinsic matrix.
        /// @param distortion_coefficients_out Receives the distortion coefficients.
        /// @return True if calibration is available, false if it has not been received yet.
        bool _get_camera_calibration(cv::Mat& camera_matrix_out, cv::Mat& distortion_coefficients_out);

        /// @brief Estimates and publishes the pose of every sufficiently large marker.
        /// @param ids Detected marker IDs.
        /// @param corners Detected marker corners, parallel to @p ids.
        /// @param header Header of the message the frame came from.
        /// @param annotated_frame Frame that pose axes are drawn onto.
        /// @return Positions of the markers that passed the bounding box area filter.
        std::vector<marker_coordinate_t> _estimate_marker_poses(
            const std::vector<int>& ids,
            const std::vector<std::vector<cv::Point2f>>& corners,
            const std_msgs::msg::Header& header,
            cv::Mat& annotated_frame);

        /// @brief Transforms one marker pose into the output frame, caches it, and broadcasts TF.
        /// @param header Header of the message the frame came from.
        /// @param marker_id ID of the detected marker.
        /// @param region_of_interest Marker bounding box in image coordinates.
        /// @param rotation_vector Rodrigues rotation vector from cv::solvePnP.
        /// @param translation_vector Translation vector from cv::solvePnP.
        void _transform_and_publish_marker(
            const std_msgs::msg::Header& header,
            int32_t marker_id,
            const sensor_msgs::msg::RegionOfInterest& region_of_interest,
            const cv::Vec3d& rotation_vector,
            const cv::Vec3d& translation_vector);

        /// @brief Publishes the cached detections as rviz visualization markers.
        /// @param header Header of the message the frame came from.
        void _publish_marker_array(const std_msgs::msg::Header& header);

        /// @brief Publishes the cached detections on the detections topic, if enabled.
        /// @param header Header of the message the frame came from.
        void _publish_detections(const std_msgs::msg::Header& header);

        /// @brief Answers a detection service request from the cached detections.
        /// @param request Incoming request, which may also ask for an image capture.
        /// @param response Response populated with the cached detections.
        void _handle_detect_objects_request(
            const std::shared_ptr<DetectObjects::Request> request,
            std::shared_ptr<DetectObjects::Response> response);

        /// @brief Writes an annotated capture of the latest frame to disk.
        /// @param save_path Directory to write the image into; created if missing.
        /// @param frame Annotated frame to overlay and save. Modified in place.
        /// @param marker_coordinates Marker positions to list on the image.
        /// @param ids Marker IDs, used to build the filename.
        void _save_annotated_capture(
            const std::string& save_path,
            cv::Mat& frame,
            const std::vector<marker_coordinate_t>& marker_coordinates,
            const std::vector<int32_t>& ids) const;

        // Parameters
        double _marker_length{DEFAULT_MARKER_LENGTH_M};
        double _axis_length{DEFAULT_AXIS_LENGTH_M};
        double _min_bounding_box_area{DEFAULT_MIN_BOUNDING_BOX_AREA_PX};
        int32_t _dictionary_id{DEFAULT_DICTIONARY_ID};

        std::string _camera_frame{DEFAULT_CAMERA_FRAME};
        std::string _tf_output_frame{DEFAULT_TF_OUTPUT_FRAME};

        std::string _input_image_topic{DEFAULT_INPUT_IMAGE_TOPIC};
        std::string _output_image_topic{DEFAULT_OUTPUT_IMAGE_TOPIC};
        std::string _output_detections_topic{DEFAULT_OUTPUT_DETECTIONS_TOPIC};
        std::string _camera_info_topic{DEFAULT_CAMERA_INFO_TOPIC};

        bool _should_publish_tf{true};
        bool _should_publish_image{true};
        bool _is_compressed_io{false};
        bool _should_use_camera_info{false};
        bool _should_publish_output{false};

        // Camera intrinsics
        cv::Mat _camera_matrix;
        cv::Mat _distortion_coefficients;

        // ArUco
        cv::aruco::ArucoDetector _detector;

        // TF
        std::unique_ptr<tf2_ros::TransformBroadcaster> _tf_broadcaster;
        std::unique_ptr<tf2_ros::Buffer> _tf_buffer;
        std::unique_ptr<tf2_ros::TransformListener> _tf_listener;

        // ROS IO
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _image_subscription;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _image_publisher;

        rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr _compressed_image_subscription;
        rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr _compressed_image_publisher;

        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr _camera_info_subscription;

        rclcpp::Service<DetectObjects>::SharedPtr _detect_objects_service;

        rclcpp::Publisher<perseus_interfaces::msg::ObjectDetections>::SharedPtr _detection_publisher;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _marker_array_publisher;

        // Cached detections for service + topic
        std::mutex _detections_mutex;
        rclcpp::Time _latest_timestamp{0, 0, RCL_ROS_TIME};
        std::vector<int32_t> _latest_ids;
        std::vector<geometry_msgs::msg::Pose> _latest_poses;
        std::vector<sensor_msgs::msg::RegionOfInterest> _latest_regions_of_interest;
        cv::Mat _latest_frame;
        std::vector<marker_coordinate_t> _latest_marker_coordinates;

        // Camera calibration synchronization
        std::mutex _camera_matrix_mutex;
    };

}  // namespace perseus_vision
