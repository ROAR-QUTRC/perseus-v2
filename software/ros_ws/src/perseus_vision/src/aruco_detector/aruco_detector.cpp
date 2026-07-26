/// @file aruco_detector.cpp
/// @brief Implementation of the ArUco marker detection and pose estimation node.

#include "perseus_vision/aruco_detector/aruco_detector.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <string>
#include <vector>

namespace perseus_vision
{
    namespace
    {
        /// @brief Queue depth used for all image, detection, and marker topics.
        constexpr int QOS_DEPTH = 10;
        /// @brief JPEG quality used when re-encoding annotated compressed images.
        constexpr int JPEG_QUALITY = 90;
        /// @brief Thickness of the flat cube drawn for each marker in rviz, in metres.
        constexpr double MARKER_VISUAL_THICKNESS_M = 0.01;
        /// @brief How long rviz keeps a marker before treating it as stale, in seconds.
        constexpr int32_t MARKER_LIFETIME_S = 2;
        /// @brief Topic that rviz visualization markers are published on.
        const std::string MARKER_ARRAY_TOPIC = "/detection/aruco/markers";

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

        /// @brief Builds the smallest axis-aligned region of interest containing all corners.
        /// @param corner_points Marker corners in image coordinates.
        /// @return The enclosing region of interest, or a zeroed one if @p corner_points is empty.
        sensor_msgs::msg::RegionOfInterest region_of_interest_from_corners(
            const std::vector<cv::Point2f>& corner_points)
        {
            sensor_msgs::msg::RegionOfInterest region_of_interest;
            if (corner_points.empty())
            {
                return region_of_interest;
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

            region_of_interest.x_offset = static_cast<uint32_t>(std::max(0.0, std::floor(min_x)));
            region_of_interest.y_offset = static_cast<uint32_t>(std::max(0.0, std::floor(min_y)));
            region_of_interest.width = static_cast<uint32_t>(std::max(0.0, std::ceil(max_x - min_x)));
            region_of_interest.height = static_cast<uint32_t>(std::max(0.0, std::ceil(max_y - min_y)));
            region_of_interest.do_rectify = false;
            return region_of_interest;
        }

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

    ArucoDetector::ArucoDetector()
        : Node("aruco_detector")
    {
        // Declare and load parameters
        _marker_length = this->declare_parameter<double>("marker_length", DEFAULT_MARKER_LENGTH_M);
        _axis_length = this->declare_parameter<double>("axis_length", DEFAULT_AXIS_LENGTH_M);
        _dictionary_id = this->declare_parameter<int>("dictionary_id", DEFAULT_DICTIONARY_ID);
        _camera_frame = this->declare_parameter<std::string>("camera_frame", DEFAULT_CAMERA_FRAME);
        _tf_output_frame = this->declare_parameter<std::string>("tf_output_frame", DEFAULT_TF_OUTPUT_FRAME);
        _input_image_topic = this->declare_parameter<std::string>("input_img", DEFAULT_INPUT_IMAGE_TOPIC);
        _output_image_topic = this->declare_parameter<std::string>("output_img", DEFAULT_OUTPUT_IMAGE_TOPIC);
        _should_publish_tf = this->declare_parameter<bool>("publish_tf", true);
        _should_publish_image = this->declare_parameter<bool>("publish_img", true);
        _is_compressed_io = this->declare_parameter<bool>("compressed_io", false);
        _should_publish_output = this->declare_parameter<bool>("publish_output", false);
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

        // TF broadcaster and listener
        _tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        _tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        _tf_listener = std::make_unique<tf2_ros::TransformListener>(*_tf_buffer);

        // Image subscriber and publisher
        if (_is_compressed_io)
        {
            _compressed_image_subscription = this->create_subscription<sensor_msgs::msg::CompressedImage>(
                _input_image_topic + "/compressed", QOS_DEPTH,
                std::bind(&ArucoDetector::_compressed_image_callback, this, std::placeholders::_1));
            _compressed_image_publisher = this->create_publisher<sensor_msgs::msg::CompressedImage>(
                _output_image_topic + "/compressed", QOS_DEPTH);
        }
        else
        {
            _image_subscription = this->create_subscription<sensor_msgs::msg::Image>(
                _input_image_topic, QOS_DEPTH,
                std::bind(&ArucoDetector::_image_callback, this, std::placeholders::_1));
            _image_publisher = this->create_publisher<sensor_msgs::msg::Image>(
                _output_image_topic, QOS_DEPTH);
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
        if (_should_publish_output)
        {
            _detection_publisher = this->create_publisher<perseus_interfaces::msg::ObjectDetections>(
                _output_detections_topic, QOS_DEPTH);
        }

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

        if (_should_publish_image)
        {
            std::lock_guard<std::mutex> lock(_detections_mutex);
            const auto processed_msg = cv_bridge::CvImage(msg->header, "bgr8", _latest_frame).toImageMsg();
            _image_publisher->publish(*processed_msg);
        }
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

        if (_should_publish_image)
        {
            sensor_msgs::msg::CompressedImage compressed_msg;
            compressed_msg.header = msg->header;
            compressed_msg.format = "jpeg";

            {
                std::lock_guard<std::mutex> lock(_detections_mutex);
                std::vector<uchar> buffer;
                const std::vector<int> encode_params = {cv::IMWRITE_JPEG_QUALITY, JPEG_QUALITY};
                cv::imencode(".jpg", _latest_frame, buffer, encode_params);
                compressed_msg.data = buffer;
            }

            _compressed_image_publisher->publish(compressed_msg);
        }
    }

    void ArucoDetector::_process_image(const cv::Mat& frame, const std_msgs::msg::Header& header)
    {
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        _detector.detectMarkers(frame, corners, ids);

        // Work with a copy for annotation
        cv::Mat annotated_frame = frame.clone();

        _reset_latest_detections(header.stamp);

        std::vector<marker_coordinate_t> marker_coordinates;
        if (!ids.empty())
        {
            cv::aruco::drawDetectedMarkers(annotated_frame, corners, ids);
            marker_coordinates = _estimate_marker_poses(ids, corners, header, annotated_frame);
        }

        // Store annotated frame for capture service
        {
            std::lock_guard<std::mutex> lock(_detections_mutex);
            _latest_frame = std::move(annotated_frame);
            _latest_marker_coordinates = marker_coordinates;
        }

        _publish_marker_array(header);
        _publish_detections(header);
    }

    void ArucoDetector::_reset_latest_detections(const rclcpp::Time& stamp)
    {
        std::lock_guard<std::mutex> lock(_detections_mutex);
        _latest_ids.clear();
        _latest_poses.clear();
        _latest_regions_of_interest.clear();
        _latest_timestamp = stamp;
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
        cv::Mat& annotated_frame)
    {
        std::vector<marker_coordinate_t> marker_coordinates;

        cv::Mat camera_matrix;
        cv::Mat distortion_coefficients;
        if (!_get_camera_calibration(camera_matrix, distortion_coefficients))
        {
            return marker_coordinates;
        }

        // Marker corner points in 3D (square marker with given length)
        const std::vector<cv::Point3f> object_points = marker_object_points(_marker_length);

        for (std::size_t i = 0; i < ids.size(); ++i)
        {
            const std::vector<cv::Point2f>& image_points = corners[i];

            cv::Vec3d rotation_vector;
            cv::Vec3d translation_vector;
            cv::solvePnP(object_points, image_points, camera_matrix, distortion_coefficients,
                         rotation_vector, translation_vector);

            // Skip detections with bounding box area smaller than threshold
            const double area = bounding_box_area(image_points);
            if (area < _min_bounding_box_area)
            {
                RCLCPP_DEBUG(this->get_logger(),
                             "Filtered out marker %d: area %.1f < min_area %.1f",
                             ids[i], area, _min_bounding_box_area);
                continue;
            }

            cv::drawFrameAxes(
                annotated_frame,
                camera_matrix, distortion_coefficients,
                rotation_vector, translation_vector,
                static_cast<float>(_axis_length));

            // Store marker position for display, converted to a forward/left/up frame
            marker_coordinates.push_back(
                {static_cast<int32_t>(ids[i]),
                 cv::Point3d(translation_vector[2], -translation_vector[0], -translation_vector[1])});

            _transform_and_publish_marker(header,
                                          static_cast<int32_t>(ids[i]),
                                          region_of_interest_from_corners(image_points),
                                          rotation_vector,
                                          translation_vector);
        }

        return marker_coordinates;
    }

    void ArucoDetector::_transform_and_publish_marker(const std_msgs::msg::Header& header,
                                                      int32_t marker_id,
                                                      const sensor_msgs::msg::RegionOfInterest& region_of_interest,
                                                      const cv::Vec3d& rotation_vector,
                                                      const cv::Vec3d& translation_vector)
    {
        try
        {
            geometry_msgs::msg::PoseStamped marker_pose_camera;
            marker_pose_camera.header.stamp = header.stamp;
            marker_pose_camera.header.frame_id = _camera_frame;

            marker_pose_camera.pose.position.x = translation_vector[0];
            marker_pose_camera.pose.position.y = translation_vector[1];
            marker_pose_camera.pose.position.z = translation_vector[2];

            // Convert rotation vector to quaternion via tf2::Matrix3x3
            cv::Mat rotation_matrix;
            cv::Rodrigues(rotation_vector, rotation_matrix);

            const tf2::Matrix3x3 tf2_rotation(
                rotation_matrix.at<double>(0, 0), rotation_matrix.at<double>(0, 1), rotation_matrix.at<double>(0, 2),
                rotation_matrix.at<double>(1, 0), rotation_matrix.at<double>(1, 1), rotation_matrix.at<double>(1, 2),
                rotation_matrix.at<double>(2, 0), rotation_matrix.at<double>(2, 1), rotation_matrix.at<double>(2, 2));

            tf2::Quaternion quaternion;
            tf2_rotation.getRotation(quaternion);

            marker_pose_camera.pose.orientation.x = quaternion.x();
            marker_pose_camera.pose.orientation.y = quaternion.y();
            marker_pose_camera.pose.orientation.z = quaternion.z();
            marker_pose_camera.pose.orientation.w = quaternion.w();

            geometry_msgs::msg::PoseStamped marker_pose_out;
            _tf_buffer->transform(marker_pose_camera, marker_pose_out, _tf_output_frame);

            // Cache this detection for service requests
            {
                std::lock_guard<std::mutex> lock(_detections_mutex);
                _latest_ids.push_back(marker_id);
                _latest_poses.push_back(marker_pose_out.pose);
                _latest_regions_of_interest.push_back(region_of_interest);
            }

            geometry_msgs::msg::TransformStamped transform;
            transform.header.stamp = header.stamp;
            transform.header.frame_id = _tf_output_frame;
            transform.child_frame_id = "aruco_marker_" + std::to_string(marker_id);

            transform.transform.translation.x = marker_pose_out.pose.position.x;
            transform.transform.translation.y = marker_pose_out.pose.position.y;
            transform.transform.translation.z = marker_pose_out.pose.position.z;

            transform.transform.rotation = marker_pose_out.pose.orientation;

            if (_should_publish_tf)
            {
                _tf_broadcaster->sendTransform(transform);
            }

            RCLCPP_DEBUG(this->get_logger(),
                         "ArUco %d in %s: x=%.2f, y=%.2f, z=%.2f",
                         marker_id, _tf_output_frame.c_str(),
                         marker_pose_out.pose.position.x,
                         marker_pose_out.pose.position.y,
                         marker_pose_out.pose.position.z);
        }
        catch (const tf2::TransformException& ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not transform marker pose: %s", ex.what());
        }
    }

    void ArucoDetector::_publish_marker_array(const std_msgs::msg::Header& header)
    {
        visualization_msgs::msg::MarkerArray marker_array;
        {
            std::lock_guard<std::mutex> lock(_detections_mutex);
            for (std::size_t i = 0; i < _latest_ids.size(); ++i)
            {
                visualization_msgs::msg::Marker marker;
                marker.header.frame_id = _tf_output_frame;
                marker.header.stamp = header.stamp;
                marker.ns = "aruco_markers";
                marker.id = _latest_ids[i];
                marker.type = visualization_msgs::msg::Marker::CUBE;
                marker.action = visualization_msgs::msg::Marker::ADD;

                marker.pose.position.x = _latest_poses[i].position.x;
                marker.pose.position.y = _latest_poses[i].position.y;
                marker.pose.position.z = _latest_poses[i].position.z;
                marker.pose.orientation = _latest_poses[i].orientation;

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

    void ArucoDetector::_publish_detections(const std_msgs::msg::Header& header)
    {
        if (!_should_publish_output || !_detection_publisher)
        {
            return;
        }

        perseus_interfaces::msg::ObjectDetections detection_msg;
        detection_msg.stamp = header.stamp;
        detection_msg.frame_id = _tf_output_frame;
        {
            std::lock_guard<std::mutex> lock(_detections_mutex);
            detection_msg.ids = _latest_ids;
            detection_msg.poses = _latest_poses;
            detection_msg.regions_of_interest = _latest_regions_of_interest;
        }
        _detection_publisher->publish(detection_msg);
    }

    void ArucoDetector::_handle_detect_objects_request(const std::shared_ptr<DetectObjects::Request> request,
                                                       std::shared_ptr<DetectObjects::Response> response)
    {
        // Get data snapshot while holding lock, then release before I/O
        cv::Mat frame_to_save;
        std::vector<marker_coordinate_t> marker_coordinates;
        std::vector<int32_t> ids;
        std::size_t detection_count = 0;
        {
            std::lock_guard<std::mutex> lock(_detections_mutex);

            response->stamp = _latest_timestamp;
            response->frame_id = _tf_output_frame;
            response->ids = _latest_ids;
            response->poses = _latest_poses;
            response->message = _latest_ids.empty()
                                    ? "No ArUco detections are currently cached."
                                    : "Returned cached ArUco detections.";
            detection_count = _latest_ids.size();

            // Copy data needed for image processing
            if (request->capture_image)
            {
                frame_to_save = _latest_frame.clone();
                marker_coordinates = _latest_marker_coordinates;
                ids = _latest_ids;
            }
        }

        // Handle image capture if requested (outside of lock)
        if (request->capture_image)
        {
            _save_annotated_capture(request->img_save_path, frame_to_save, marker_coordinates, ids);
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
                                                const std::vector<marker_coordinate_t>& marker_coordinates,
                                                const std::vector<int32_t>& ids) const
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

            draw_capture_overlay(frame, marker_coordinates);

            // Add marker IDs and a timestamp to the filename to prevent overwrites
            const auto epoch_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                      std::chrono::system_clock::now().time_since_epoch())
                                      .count();
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
