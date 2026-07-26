/// @file aruco_detector.cpp
/// @brief Implementation of the ArUco marker detection and pose estimation node.

#include "perseus_vision/aruco_detector/aruco_detector.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <rclcpp_components/register_node_macro.hpp>
#include <string>
#include <vector>

#include "perseus_vision/common/detection_renderer.hpp"

namespace perseus_vision
{
    namespace
    {
        /// @brief Queue depth used for all image, detection, and marker topics.
        constexpr int QOS_DEPTH = 10;
        /// @brief Thickness of the flat cube drawn for each marker in rviz, in metres.
        constexpr double MARKER_VISUAL_THICKNESS_M = 0.01;
        /// @brief How long rviz keeps a marker before treating it as stale, in seconds.
        constexpr int32_t MARKER_LIFETIME_S = 2;
        /// @brief Topic that rviz visualization markers are published on.
        const std::string MARKER_ARRAY_TOPIC = "/detection/aruco/markers";
        /// @brief Colour the overlay node draws ArUco boxes in, in OpenCV BGR order.
        const cv::Scalar DETECTION_COLOR(0, 255, 0);
        /// @brief Confidence reported for ArUco markers.
        ///
        /// Detection is geometric rather than probabilistic: a marker either decodes to a
        /// valid ID or it is not reported at all, so an accepted marker is always 1.0.
        constexpr float DETECTION_CONFIDENCE = 1.0f;

        /// @brief Default 3x3 intrinsic matrix, in row-major order, used until calibration arrives.
        const std::vector<double> DEFAULT_CAMERA_MATRIX = {530.4, 0.0, 320.0, 0.0, 530.4, 240.0, 0.0, 0.0, 1.0};
        /// @brief Default distortion coefficients, assuming an ideal pinhole camera.
        const std::vector<double> DEFAULT_DISTORTION_COEFFICIENTS = {0.0, 0.0, 0.0, 0.0, 0.0};
        /// @brief Number of elements in a row-major 3x3 intrinsic matrix.
        constexpr std::size_t CAMERA_MATRIX_ELEMENT_COUNT = 9;
        /// @brief Side length of a row or column of the intrinsic matrix.
        constexpr int CAMERA_MATRIX_SIDE = 3;

        /// @brief Left margin used for all capture overlay text, in pixels.
        constexpr int OVERLAY_MARGIN_PX = 10;
        /// @brief Baseline of the capture timestamp line, in pixels from the top.
        constexpr int OVERLAY_TIMESTAMP_Y_PX = 30;
        /// @brief Baseline of the marker coordinate heading, in pixels from the top.
        constexpr int OVERLAY_HEADING_Y_PX = 70;
        /// @brief Vertical spacing between marker coordinate lines, in pixels.
        constexpr int OVERLAY_LINE_HEIGHT_PX = 25;
        /// @brief Font scale used for the capture timestamp.
        constexpr double OVERLAY_TIMESTAMP_FONT_SCALE = 0.6;
        /// @brief Font scale used for marker coordinate text.
        constexpr double OVERLAY_TEXT_FONT_SCALE = 0.5;
        /// @brief Stroke thickness used for the capture timestamp.
        constexpr int OVERLAY_TIMESTAMP_THICKNESS = 2;
        /// @brief Stroke thickness used for marker coordinate text.
        constexpr int OVERLAY_TEXT_THICKNESS = 1;
        /// @brief Longest formatted marker coordinate line, including the terminator.
        constexpr std::size_t COORDINATE_TEXT_BUFFER_SIZE = 150;
        /// @brief Longest formatted timestamp, including the terminator.
        constexpr std::size_t TIMESTAMP_BUFFER_SIZE = 100;

        /// @brief Computes the area of the axis-aligned box enclosing a set of corners.
        /// @param corner_points Marker corners in image coordinates.
        /// @return The bounding box area in square pixels, or 0 if @p corner_points is empty.
        double bounding_box_area(const std::vector<cv::Point2f>& corner_points)
        {
            if (corner_points.empty())
            {
                return 0.0;
            }

            double min_x = corner_points.front().x;
            double max_x = corner_points.front().x;
            double min_y = corner_points.front().y;
            double max_y = corner_points.front().y;

            for (const auto& corner_point : corner_points)
            {
                min_x = std::min(min_x, static_cast<double>(corner_point.x));
                max_x = std::max(max_x, static_cast<double>(corner_point.x));
                min_y = std::min(min_y, static_cast<double>(corner_point.y));
                max_y = std::max(max_y, static_cast<double>(corner_point.y));
            }

            return (max_x - min_x) * (max_y - min_y);
        }

        /// @brief Builds the 3D corner points of a square marker, centred on its own origin.
        /// @param marker_length Marker side length in metres.
        /// @return The four corners in the order OpenCV reports detected corners.
        std::vector<cv::Point3f> marker_object_points(double marker_length)
        {
            const float half_length = static_cast<float>(marker_length / 2.0);
            return {
                cv::Point3f(-half_length, half_length, 0.0f),
                cv::Point3f(half_length, half_length, 0.0f),
                cv::Point3f(half_length, -half_length, 0.0f),
                cv::Point3f(-half_length, -half_length, 0.0f)};
        }

        /// @brief Converts a cv::solvePnP result into a pose in the camera frame.
        /// @param rotation_vector Rodrigues rotation vector from cv::solvePnP.
        /// @param translation_vector Translation vector from cv::solvePnP.
        /// @return The equivalent pose.
        geometry_msgs::msg::Pose pose_from_rodrigues(const cv::Vec3d& rotation_vector,
                                                     const cv::Vec3d& translation_vector)
        {
            cv::Mat rotation_matrix;
            cv::Rodrigues(rotation_vector, rotation_matrix);

            const tf2::Matrix3x3 tf2_rotation(
                rotation_matrix.at<double>(0, 0), rotation_matrix.at<double>(0, 1), rotation_matrix.at<double>(0, 2),
                rotation_matrix.at<double>(1, 0), rotation_matrix.at<double>(1, 1), rotation_matrix.at<double>(1, 2),
                rotation_matrix.at<double>(2, 0), rotation_matrix.at<double>(2, 1), rotation_matrix.at<double>(2, 2));

            tf2::Quaternion quaternion;
            tf2_rotation.getRotation(quaternion);

            geometry_msgs::msg::Pose pose;
            pose.position.x = translation_vector[0];
            pose.position.y = translation_vector[1];
            pose.position.z = translation_vector[2];
            pose.orientation.x = quaternion.x();
            pose.orientation.y = quaternion.y();
            pose.orientation.z = quaternion.z();
            pose.orientation.w = quaternion.w();
            return pose;
        }

        /// @brief Formats the current local time for display on a captured image.
        /// @return The time formatted as `YYYY-MM-DD HH:MM:SS`.
        std::string format_local_timestamp()
        {
            const auto now = std::chrono::system_clock::now();
            const std::time_t now_seconds = std::chrono::system_clock::to_time_t(now);

            // localtime_r is used over localtime as the latter is not thread safe.
            std::tm local_time{};
            localtime_r(&now_seconds, &local_time);

            char timestamp_text[TIMESTAMP_BUFFER_SIZE];
            std::strftime(timestamp_text, sizeof(timestamp_text), "%Y-%m-%d %H:%M:%S", &local_time);
            return std::string(timestamp_text);
        }

        /// @brief Builds the marker ID portion of a capture filename.
        /// @param ids Marker IDs present in the capture.
        /// @return The IDs joined with underscores, or `no_markers` if @p ids is empty.
        std::string join_marker_ids(const std::vector<int32_t>& ids)
        {
            if (ids.empty())
            {
                return "no_markers";
            }

            std::string joined_ids;
            for (std::size_t i = 0; i < ids.size(); ++i)
            {
                if (i > 0)
                {
                    joined_ids += "_";
                }
                joined_ids += std::to_string(ids[i]);
            }
            return joined_ids;
        }

        /// @brief Draws the capture timestamp and marker coordinates onto an image.
        /// @param frame Image to draw onto. Modified in place.
        /// @param marker_coordinates Marker positions to list under the timestamp.
        void draw_capture_overlay(cv::Mat& frame, const std::vector<marker_coordinate_t>& marker_coordinates)
        {
            const cv::Scalar timestamp_color(0, 255, 0);
            const cv::Scalar heading_color(0, 255, 255);
            const cv::Scalar coordinate_color(255, 255, 0);

            cv::putText(frame, "Time: " + format_local_timestamp(),
                        cv::Point(OVERLAY_MARGIN_PX, OVERLAY_TIMESTAMP_Y_PX),
                        cv::FONT_HERSHEY_SIMPLEX, OVERLAY_TIMESTAMP_FONT_SCALE,
                        timestamp_color, OVERLAY_TIMESTAMP_THICKNESS);

            if (marker_coordinates.empty())
            {
                return;
            }

            int text_y = OVERLAY_HEADING_Y_PX;
            cv::putText(frame, "Marker Coordinates (XYZ):",
                        cv::Point(OVERLAY_MARGIN_PX, text_y),
                        cv::FONT_HERSHEY_SIMPLEX, OVERLAY_TEXT_FONT_SCALE,
                        heading_color, OVERLAY_TEXT_THICKNESS);

            for (const auto& marker_coordinate : marker_coordinates)
            {
                text_y += OVERLAY_LINE_HEIGHT_PX;
                char coordinate_text[COORDINATE_TEXT_BUFFER_SIZE];
                std::snprintf(coordinate_text, sizeof(coordinate_text),
                              "ID %d: X=%.3f, Y=%.3f, Z=%.3f",
                              marker_coordinate.id,
                              marker_coordinate.position.x,
                              marker_coordinate.position.y,
                              marker_coordinate.position.z);

                cv::putText(frame, std::string(coordinate_text),
                            cv::Point(OVERLAY_MARGIN_PX, text_y),
                            cv::FONT_HERSHEY_SIMPLEX, OVERLAY_TEXT_FONT_SCALE,
                            coordinate_color, OVERLAY_TEXT_THICKNESS);
            }
        }
    }  // namespace

    ArucoDetector::ArucoDetector(const rclcpp::NodeOptions& options)
        : Node("aruco_detector", options)
    {
        // Declare and load parameters
        _marker_length = this->declare_parameter<double>("marker_length", DEFAULT_MARKER_LENGTH_M);
        _dictionary_id = this->declare_parameter<int>("dictionary_id", DEFAULT_DICTIONARY_ID);
        _camera_frame = this->declare_parameter<std::string>("camera_frame", DEFAULT_CAMERA_FRAME);
        _input_image_topic = this->declare_parameter<std::string>("input_img", DEFAULT_INPUT_IMAGE_TOPIC);
        _output_detections_topic = this->declare_parameter<std::string>(
            "output_detections_2d_topic", DEFAULT_OUTPUT_DETECTIONS_TOPIC);
        _should_publish_tf = this->declare_parameter<bool>("publish_tf", true);
        _is_compressed_io = this->declare_parameter<bool>("compressed_io", false);
        _should_use_camera_info = this->declare_parameter<bool>("use_camera_info", false);
        _output_detections_topic = this->declare_parameter<std::string>("output_topic", DEFAULT_OUTPUT_DETECTIONS_TOPIC);
        _camera_info_topic = this->declare_parameter<std::string>("camera_info_topic", DEFAULT_CAMERA_INFO_TOPIC);
        _min_bounding_box_area = this->declare_parameter<double>("min_bounding_box_area", DEFAULT_MIN_BOUNDING_BOX_AREA_PX);

        std::vector<double> camera_matrix_param =
            this->declare_parameter<std::vector<double>>("camera_matrix", DEFAULT_CAMERA_MATRIX);
        const std::vector<double> distortion_coefficients_param =
            this->declare_parameter<std::vector<double>>("distortion_coefficients", DEFAULT_DISTORTION_COEFFICIENTS);

        // Validate and convert camera matrix parameter
        if (camera_matrix_param.size() != CAMERA_MATRIX_ELEMENT_COUNT)
        {
            RCLCPP_ERROR(this->get_logger(),
                         "camera_matrix must have exactly 9 elements (3x3 row-major), got %zu. Using defaults.",
                         camera_matrix_param.size());
            camera_matrix_param = DEFAULT_CAMERA_MATRIX;
        }

        _camera_matrix = cv::Mat(CAMERA_MATRIX_SIDE, CAMERA_MATRIX_SIDE, CV_64F);
        for (std::size_t i = 0; i < CAMERA_MATRIX_ELEMENT_COUNT; ++i)
        {
            const int row = static_cast<int>(i) / CAMERA_MATRIX_SIDE;
            const int column = static_cast<int>(i) % CAMERA_MATRIX_SIDE;
            _camera_matrix.at<double>(row, column) = camera_matrix_param[i];
        }

        _distortion_coefficients = cv::Mat(static_cast<int>(distortion_coefficients_param.size()), 1, CV_64F);
        for (std::size_t i = 0; i < distortion_coefficients_param.size(); ++i)
        {
            _distortion_coefficients.at<double>(static_cast<int>(i), 0) = distortion_coefficients_param[i];
        }

        // ArUco setup
        const cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(_dictionary_id);
        _detector = cv::aruco::ArucoDetector(dictionary);

        // TF broadcaster
        _tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // Image subscriber and publisher
        if (_is_compressed_io)
        {
            _compressed_image_subscription = this->create_subscription<sensor_msgs::msg::CompressedImage>(
                _input_image_topic + "/compressed", QOS_DEPTH,
                std::bind(&ArucoDetector::_compressed_image_callback, this, std::placeholders::_1));
        }
        else
        {
            _image_subscription = this->create_subscription<sensor_msgs::msg::Image>(
                _input_image_topic, QOS_DEPTH,
                std::bind(&ArucoDetector::_image_callback, this, std::placeholders::_1));
        }

        // Camera info subscriber if enabled (works for both raw and compressed modes)
        if (_should_use_camera_info)
        {
            _camera_info_subscription = this->create_subscription<sensor_msgs::msg::CameraInfo>(
                _camera_info_topic,
                rclcpp::SensorDataQoS(),
                std::bind(&ArucoDetector::_camera_info_callback, this, std::placeholders::_1));

            RCLCPP_INFO(this->get_logger(), "Subscribing to camera_info from topic: %s", _camera_info_topic.c_str());
        }

        _detect_objects_service = this->create_service<DetectObjects>(
            "detect_objects",
            std::bind(&ArucoDetector::_handle_detect_objects_request, this,
                      std::placeholders::_1,
                      std::placeholders::_2));
        _detection_publisher = this->create_publisher<perseus_interfaces::msg::DetectionArray>(
            _output_detections_topic, QOS_DEPTH);

        // Create marker array publisher for visualization
        _marker_array_publisher = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            MARKER_ARRAY_TOPIC, QOS_DEPTH);

        RCLCPP_INFO(this->get_logger(), "Perseus' ArucoDetector node started.");
    }

    void ArucoDetector::_image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv::Mat frame;
        try
        {
            frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
        }
        catch (const cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        _process_image(frame, msg->header);
    }

    void ArucoDetector::_compressed_image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
    {
        cv::Mat frame;
        try
        {
            frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
            if (frame.empty())
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to decode compressed image");
                return;
            }
        }
        catch (const cv::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "OpenCV exception: %s", e.what());
            return;
        }

        _process_image(frame, msg->header);
    }

    void ArucoDetector::_process_image(const cv::Mat& frame, const std_msgs::msg::Header& header)
    {
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        _detector.detectMarkers(frame, corners, ids);

        // Detections are published rather than drawn here: the overlay node owns
        // annotation, so nothing is rendered on the live detection path.
        perseus_interfaces::msg::DetectionArray detections;
        detections.header = header;

        std::vector<marker_coordinate_t> marker_coordinates;
        if (!ids.empty())
        {
            marker_coordinates = _estimate_marker_poses(ids, corners, header, detections);
        }

        // Store the raw frame so the capture service can annotate it on demand
        {
            std::lock_guard<std::mutex> lock(_detections_mutex);
            _latest_frame = frame.clone();
            _latest_detections = detections;
            _latest_marker_coordinates = marker_coordinates;
        }

        _publish_marker_array(detections);
        _publish_detections(detections);
    }

    bool ArucoDetector::_get_camera_calibration(cv::Mat& camera_matrix_out, cv::Mat& distortion_coefficients_out)
    {
        std::lock_guard<std::mutex> lock(_camera_matrix_mutex);
        if (_camera_matrix.empty())
        {
            RCLCPP_WARN_ONCE(this->get_logger(), "Camera matrix not initialized, skipping pose estimation");
            return false;
        }

        camera_matrix_out = _camera_matrix.clone();
        distortion_coefficients_out = _distortion_coefficients.clone();
        return true;
    }

    std::vector<marker_coordinate_t> ArucoDetector::_estimate_marker_poses(
        const std::vector<int>& ids,
        const std::vector<std::vector<cv::Point2f>>& corners,
        const std_msgs::msg::Header& header,
        perseus_interfaces::msg::DetectionArray& detections_out)
    {
        std::vector<marker_coordinate_t> marker_coordinates;

        // Pose estimation needs intrinsics, but image-space detection does not, so a
        // missing calibration only disables the 3D half of this function.
        cv::Mat camera_matrix;
        cv::Mat distortion_coefficients;
        const bool has_calibration = _get_camera_calibration(camera_matrix, distortion_coefficients);

        // Marker corner points in 3D (square marker with given length)
        const std::vector<cv::Point3f> object_points = marker_object_points(_marker_length);

        for (std::size_t i = 0; i < ids.size(); ++i)
        {
            const std::vector<cv::Point2f>& image_points = corners[i];

            // Skip detections with bounding box area smaller than threshold. This is
            // checked before solving for the pose so filtered markers cost nothing.
            const double area = bounding_box_area(image_points);
            if (area < _min_bounding_box_area)
            {
                RCLCPP_DEBUG(this->get_logger(),
                             "Filtered out marker %d: area %.1f < min_area %.1f",
                             ids[i], area, _min_bounding_box_area);
                continue;
            }

            const int32_t marker_id = static_cast<int32_t>(ids[i]);

            perseus_interfaces::msg::Detection detection;
            detection.bounding_box = polygon_from_corners(image_points);
            detection.color = color_from_bgr(DETECTION_COLOR);
            detection.class_label = "aruco_" + std::to_string(marker_id);
            detection.id = marker_id;
            detection.confidence = DETECTION_CONFIDENCE;
            detection.has_pose = false;

            if (has_calibration)
            {
                cv::Vec3d rotation_vector;
                cv::Vec3d translation_vector;
                cv::solvePnP(object_points, image_points, camera_matrix, distortion_coefficients,
                             rotation_vector, translation_vector);

                detection.pose = pose_from_rodrigues(rotation_vector, translation_vector);
                detection.has_pose = true;

                // Store marker position for display, converted to a forward/left/up frame
                marker_coordinates.push_back(
                    {marker_id,
                     cv::Point3d(translation_vector[2], -translation_vector[0], -translation_vector[1])});

                _broadcast_marker_tf(header, marker_id, detection.pose);
            }

            detections_out.detections.push_back(std::move(detection));
        }

        return marker_coordinates;
    }

    void ArucoDetector::_broadcast_marker_tf(const std_msgs::msg::Header& header,
                                             int32_t marker_id,
                                             const geometry_msgs::msg::Pose& pose)
    {
        if (!_should_publish_tf)
        {
            return;
        }

        // Broadcast the transform that was actually measured, camera to marker, and let
        // tf2 compose it with the rest of the tree. Transforming into odom here would
        // both duplicate what tf2 already does and bake in this node's lookup time.
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = header.stamp;
        transform.header.frame_id = _camera_frame;
        transform.child_frame_id = "aruco_marker_" + std::to_string(marker_id);

        transform.transform.translation.x = pose.position.x;
        transform.transform.translation.y = pose.position.y;
        transform.transform.translation.z = pose.position.z;
        transform.transform.rotation = pose.orientation;

        _tf_broadcaster->sendTransform(transform);

        RCLCPP_DEBUG(this->get_logger(),
                     "ArUco %d in %s: x=%.2f, y=%.2f, z=%.2f",
                     marker_id, _camera_frame.c_str(),
                     pose.position.x, pose.position.y, pose.position.z);
    }

    void ArucoDetector::_publish_marker_array(const perseus_interfaces::msg::DetectionArray& detections)
    {
        visualization_msgs::msg::MarkerArray marker_array;
        {
            for (const auto& detection : detections.detections)
            {
                if (!detection.has_pose)
                {
                    continue;
                }

                visualization_msgs::msg::Marker marker;
                // Markers are drawn in the camera frame, the frame the poses are measured
                // in. rviz composes the rest of the tree itself.
                marker.header.frame_id = _camera_frame;
                marker.header.stamp = detections.header.stamp;
                marker.ns = "aruco_markers";
                marker.id = detection.id;
                marker.type = visualization_msgs::msg::Marker::CUBE;
                marker.action = visualization_msgs::msg::Marker::ADD;

                marker.pose = detection.pose;

                marker.scale.x = _marker_length;
                marker.scale.y = _marker_length;
                marker.scale.z = MARKER_VISUAL_THICKNESS_M;

                marker.color.r = 1.0f;
                marker.color.g = 1.0f;
                marker.color.b = 1.0f;
                marker.color.a = 1.0f;

                // Set lifetime to ensure stale markers are cleaned up
                marker.lifetime = rclcpp::Duration(MARKER_LIFETIME_S, 0);

                marker_array.markers.push_back(marker);
            }
        }
        _marker_array_publisher->publish(marker_array);
    }

    void ArucoDetector::_publish_detections(const perseus_interfaces::msg::DetectionArray& detections)
    {
        _detection_publisher->publish(detections);
    }

    void ArucoDetector::_handle_detect_objects_request(const std::shared_ptr<DetectObjects::Request> request,
                                                       std::shared_ptr<DetectObjects::Response> response)
    {
        // Get data snapshot while holding lock, then release before I/O
        cv::Mat frame_to_save;
        perseus_interfaces::msg::DetectionArray detections;
        std::vector<marker_coordinate_t> marker_coordinates;
        std::size_t detection_count = 0;
        {
            std::lock_guard<std::mutex> lock(_detections_mutex);

            response->header = _latest_detections.header;
            response->detections = _latest_detections.detections;
            response->message = _latest_detections.detections.empty()
                                    ? "No ArUco detections are currently cached."
                                    : "Returned cached ArUco detections.";
            detection_count = _latest_detections.detections.size();

            // Copy data needed for image processing
            if (request->capture_image)
            {
                frame_to_save = _latest_frame.clone();
                detections = _latest_detections;
                marker_coordinates = _latest_marker_coordinates;
            }
        }

        // Handle image capture if requested (outside of lock)
        if (request->capture_image)
        {
            _save_annotated_capture(request->img_save_path, frame_to_save, detections, marker_coordinates);
        }

        if (detection_count > 0)
        {
            RCLCPP_INFO(this->get_logger(), "Service request: returning %zu detections", detection_count);
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Service request: no detections available");
        }
    }

    void ArucoDetector::_save_annotated_capture(const std::string& save_path,
                                                cv::Mat& frame,
                                                const perseus_interfaces::msg::DetectionArray& detections,
                                                const std::vector<marker_coordinate_t>& marker_coordinates) const
    {
        if (frame.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Capture requested but no frame available");
            return;
        }

        try
        {
            // Create directory if it doesn't exist
            std::error_code error_code;
            std::filesystem::create_directories(save_path, error_code);
            if (error_code)
            {
                RCLCPP_WARN(this->get_logger(), "Failed to create directory: %s", save_path.c_str());
                return;
            }

            // Render exactly as the overlay node does, then add the capture-only text
            draw_detections(frame, detections);
            draw_capture_overlay(frame, marker_coordinates);

            // Add marker IDs and a timestamp to the filename to prevent overwrites
            const auto epoch_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                      std::chrono::system_clock::now().time_since_epoch())
                                      .count();
            std::vector<int32_t> ids;
            ids.reserve(detections.detections.size());
            for (const auto& detection : detections.detections)
            {
                ids.push_back(detection.id);
            }
            const std::string filename = save_path + "/aruco_" + join_marker_ids(ids) +
                                         "_" + std::to_string(epoch_ms) + ".png";

            if (cv::imwrite(filename, frame))
            {
                RCLCPP_INFO(this->get_logger(), "Captured annotated image saved to: %s", filename.c_str());
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to write image to: %s", filename.c_str());
            }
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "Exception during image capture: %s", e.what());
        }
    }

    void ArucoDetector::_camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(_camera_matrix_mutex);

        // Extract camera matrix K (3x3)
        _camera_matrix = cv::Mat(CAMERA_MATRIX_SIDE, CAMERA_MATRIX_SIDE, CV_64F);
        for (int row = 0; row < CAMERA_MATRIX_SIDE; ++row)
        {
            for (int column = 0; column < CAMERA_MATRIX_SIDE; ++column)
            {
                _camera_matrix.at<double>(row, column) = msg->k[(row * CAMERA_MATRIX_SIDE) + column];
            }
        }

        // Extract distortion coefficients
        _distortion_coefficients = cv::Mat(static_cast<int>(msg->d.size()), 1, CV_64F);
        for (std::size_t i = 0; i < msg->d.size(); ++i)
        {
            _distortion_coefficients.at<double>(static_cast<int>(i), 0) = msg->d[i];
        }

        RCLCPP_DEBUG(this->get_logger(), "Updated camera calibration from camera_info topic");
    }

}  // namespace perseus_vision

RCLCPP_COMPONENTS_REGISTER_NODE(perseus_vision::ArucoDetector)
