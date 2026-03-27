#include "perseus_vision/aruco_detector.hpp"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <filesystem>

namespace perseus_vision
{
    namespace
    {
        sensor_msgs::msg::RegionOfInterest roi_from_corners(const std::vector<cv::Point2f>& corner_points)
        {
            sensor_msgs::msg::RegionOfInterest roi;
            if (corner_points.empty())
            {
                return roi;
            }

            double min_x = corner_points.front().x;
            double max_x = corner_points.front().x;
            double min_y = corner_points.front().y;
            double max_y = corner_points.front().y;

            for (const auto& pt : corner_points)
            {
                min_x = std::min(min_x, static_cast<double>(pt.x));
                max_x = std::max(max_x, static_cast<double>(pt.x));
                min_y = std::min(min_y, static_cast<double>(pt.y));
                max_y = std::max(max_y, static_cast<double>(pt.y));
            }

            roi.x_offset = static_cast<uint32_t>(std::max(0.0, std::floor(min_x)));
            roi.y_offset = static_cast<uint32_t>(std::max(0.0, std::floor(min_y)));
            roi.width = static_cast<uint32_t>(std::max(0.0, std::ceil(max_x - min_x)));
            roi.height = static_cast<uint32_t>(std::max(0.0, std::ceil(max_y - min_y)));
            roi.do_rectify = false;
            return roi;
        }
    }  // namespace

    ArucoDetector::ArucoDetector()
        : Node("aruco_detector")
    {
        // Declare and load parameters
        marker_length_ = this->declare_parameter<double>("marker_length", 0.35);
        axis_length_ = this->declare_parameter<double>("axis_length", 0.03);
        dictionary_id_ = this->declare_parameter<int>("dictionary_id", 1);
        camera_frame_ = this->declare_parameter<std::string>("camera_frame", "camera_color_optical_frame");
        tf_output_frame_ = this->declare_parameter<std::string>("tf_output_frame", "odom");
        input_img_ = this->declare_parameter<std::string>("input_img", "/camera/camera/color/image_raw");
        output_img_ = this->declare_parameter<std::string>("output_img", "/perseus_vision/aruco/image");
        should_publish_tf_ = this->declare_parameter<bool>("publish_tf", true);
        should_publish_img_ = this->declare_parameter<bool>("publish_img", true);
        is_compressed_io_ = this->declare_parameter<bool>("compressed_io", false);
        should_publish_output_ = this->declare_parameter<bool>("publish_output", false);
        should_use_camera_info_ = this->declare_parameter<bool>("use_camera_info", false);
        output_topic_ = this->declare_parameter<std::string>("output_topic", "/perseus_vision/aruco/detections");
        camera_info_topic_ = this->declare_parameter<std::string>("camera_info_topic", "/camera/camera/color/camera_info");
        min_bounding_box_area_ = this->declare_parameter<double>("min_bounding_box_area", 100.0);

        std::vector<double> camera_matrix_param = this->declare_parameter<std::vector<double>>(
            "camera_matrix", {530.4, 0.0, 320.0, 0.0, 530.4, 240.0, 0.0, 0.0, 1.0});
        std::vector<double> dist_coeffs_param = this->declare_parameter<std::vector<double>>(
            "distortion_coefficients", {0.0, 0.0, 0.0, 0.0, 0.0});

        // Validate and convert camera matrix parameter
        if (camera_matrix_param.size() != 9)
        {
            RCLCPP_ERROR(this->get_logger(),
                         "camera_matrix must have exactly 9 elements (3x3 row-major), got %zu. Using defaults.",
                         camera_matrix_param.size());
            camera_matrix_param = {530.4, 0.0, 320.0, 0.0, 530.4, 240.0, 0.0, 0.0, 1.0};
        }

        camera_matrix_ = cv::Mat(3, 3, CV_64F);
        for (size_t i = 0; i < 9; ++i)
        {
            camera_matrix_.at<double>(i / 3, i % 3) = camera_matrix_param[i];
        }
        dist_coeffs_ = cv::Mat(dist_coeffs_param.size(), 1, CV_64F);
        for (size_t i = 0; i < dist_coeffs_param.size(); ++i)
        {
            dist_coeffs_.at<double>(i, 0) = dist_coeffs_param[i];
        }

        // ArUco setup
        cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(dictionary_id_);
        detector_ = cv::aruco::ArucoDetector(dictionary);

        // TF broadcaster and listener
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

        // Image subscriber and publisher
        if (is_compressed_io_)
        {
            compressed_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
                input_img_ + "/compressed", kQosDepth,
                std::bind(&ArucoDetector::compressed_image_callback, this, std::placeholders::_1));
            compressed_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
                output_img_ + "/compressed", kQosDepth);
        }
        else
        {
            sub_ = this->create_subscription<sensor_msgs::msg::Image>(
                input_img_, kQosDepth,
                std::bind(&ArucoDetector::image_callback, this, std::placeholders::_1));
            pub_ = this->create_publisher<sensor_msgs::msg::Image>(
                output_img_, kQosDepth);
        }

        // Camera info subscriber if enabled (works for both raw and compressed modes)
        if (should_use_camera_info_)
        {
            camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
                camera_info_topic_,
                rclcpp::SensorDataQoS(),
                std::bind(&ArucoDetector::camera_info_callback, this, std::placeholders::_1));

            RCLCPP_INFO(this->get_logger(), "Subscribing to camera_info from topic: %s", camera_info_topic_.c_str());
        }

        service_ = this->create_service<DetectObjects>(
            "detect_objects",
            std::bind(&ArucoDetector::handle_request, this,
                      std::placeholders::_1,
                      std::placeholders::_2));
        if (should_publish_output_)
        {
            detection_pub_ = this->create_publisher<perseus_interfaces::msg::ObjectDetections>(
                output_topic_, kQosDepth);
        }

        // Create marker array publisher for visualization
        marker_array_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/detection/aruco/markers", kQosDepth);

        RCLCPP_INFO(this->get_logger(), "Perseus' ArucoDetector node started.");
    }

    void ArucoDetector::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv::Mat frame;
        try
        {
            frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        process_image(frame, msg->header);

        if (should_publish_img_)
        {
            std::lock_guard<std::mutex> lock(detections_mutex_);
            auto processed_msg = cv_bridge::CvImage(msg->header, "bgr8", latest_frame_).toImageMsg();
            pub_->publish(*processed_msg);
        }
    }

    void ArucoDetector::compressed_image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
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
        catch (cv::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "OpenCV exception: %s", e.what());
            return;
        }

        process_image(frame, msg->header);

        if (should_publish_img_)
        {
            sensor_msgs::msg::CompressedImage compressed_msg;
            compressed_msg.header = msg->header;
            compressed_msg.format = "jpeg";

            {
                std::lock_guard<std::mutex> lock(detections_mutex_);
                std::vector<uchar> buffer;
                std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 90};
                cv::imencode(".jpg", latest_frame_, buffer, params);
                compressed_msg.data = buffer;
            }

            compressed_pub_->publish(compressed_msg);
        }
    }

    void ArucoDetector::process_image(const cv::Mat& frame, const std_msgs::msg::Header& header)
    {
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        detector_.detectMarkers(frame, corners, ids);

        // Work with a copy for annotation
        cv::Mat annotated_frame = frame.clone();

        // Clear previous detections and update timestamp for service requests
        {
            std::lock_guard<std::mutex> lock(detections_mutex_);
            latest_ids_.clear();
            latest_poses_.clear();
            latest_regions_of_interest_.clear();
            latest_timestamp_ = header.stamp;
        }

        // Store marker info for corner display
        std::vector<std::pair<int, cv::Point3d>> marker_coords;

        if (!ids.empty())
        {
            // Check if camera matrix is initialized and get safe copies
            cv::Mat local_camera_matrix, local_dist_coeffs;
            bool has_camera_matrix = false;
            {
                std::lock_guard<std::mutex> lock(camera_matrix_mutex_);
                if (!camera_matrix_.empty())
                {
                    local_camera_matrix = camera_matrix_.clone();
                    local_dist_coeffs = dist_coeffs_.clone();
                    has_camera_matrix = true;
                }
                else
                {
                    RCLCPP_WARN_ONCE(this->get_logger(), "Camera matrix not initialized, skipping pose estimation");
                }
            }

            if (has_camera_matrix)
            {
                std::vector<cv::Vec3d> rvecs, tvecs;

                cv::aruco::drawDetectedMarkers(annotated_frame, corners, ids);

                // Marker corner points in 3D (square marker with given length)
                std::vector<cv::Point3f> marker_obj_points;
                float half_length = marker_length_ / 2.0f;
                marker_obj_points.push_back(cv::Point3f(-half_length, half_length, 0));
                marker_obj_points.push_back(cv::Point3f(half_length, half_length, 0));
                marker_obj_points.push_back(cv::Point3f(half_length, -half_length, 0));
                marker_obj_points.push_back(cv::Point3f(-half_length, -half_length, 0));

                for (size_t i = 0; i < ids.size(); ++i)
                {
                    cv::Vec3d rvec, tvec;
                    std::vector<cv::Point2f> image_points(corners[i].begin(), corners[i].end());

                    cv::solvePnP(marker_obj_points, image_points, local_camera_matrix, local_dist_coeffs, rvec, tvec);
                    rvecs.push_back(rvec);
                    tvecs.push_back(tvec);

                    // Calculate bounding box area for filtering
                    const auto& corner = corners[i];
                    double min_x = corner[0].x, max_x = corner[0].x;
                    double min_y = corner[0].y, max_y = corner[0].y;

                    for (const auto& pt : corner)
                    {
                        min_x = std::min(min_x, static_cast<double>(pt.x));
                        max_x = std::max(max_x, static_cast<double>(pt.x));
                        min_y = std::min(min_y, static_cast<double>(pt.y));
                        max_y = std::max(max_y, static_cast<double>(pt.y));
                    }

                    double bbox_area = (max_x - min_x) * (max_y - min_y);

                    // Skip detections with bounding box area smaller than threshold
                    if (bbox_area < min_bounding_box_area_)
                    {
                        RCLCPP_DEBUG(this->get_logger(),
                                     "Filtered out marker %d: area %.1f < min_area %.1f",
                                     ids[i], bbox_area, min_bounding_box_area_);
                        continue;
                    }

                    cv::drawFrameAxes(
                        annotated_frame,
                        local_camera_matrix, local_dist_coeffs,
                        rvecs[i], tvecs[i],
                        axis_length_);

                    // Store marker position for display
                    cv::Point3d pos(tvecs[i][2], -tvecs[i][0], -tvecs[i][1]);
                    marker_coords.push_back({ids[i], pos});

                    transform_and_publish_marker(header, ids[i], roi_from_corners(image_points), rvecs[i], tvecs[i]);
                }
            }
            else
            {
                // Camera matrix not available, just draw detected corners
                cv::aruco::drawDetectedMarkers(annotated_frame, corners, ids);
            }
        }

        // Store annotated frame for capture service
        {
            std::lock_guard<std::mutex> lock(detections_mutex_);
            latest_frame_ = std::move(annotated_frame);
            latest_marker_coords_ = marker_coords;
        }

        // Publish marker array for visualization
        visualization_msgs::msg::MarkerArray marker_array;
        {
            std::lock_guard<std::mutex> lock(detections_mutex_);
            for (size_t i = 0; i < latest_ids_.size(); ++i)
            {
                visualization_msgs::msg::Marker marker;
                marker.header.frame_id = tf_output_frame_;
                marker.header.stamp = header.stamp;
                marker.ns = "aruco_markers";
                marker.id = latest_ids_[i];
                marker.type = visualization_msgs::msg::Marker::CUBE;
                marker.action = visualization_msgs::msg::Marker::ADD;

                marker.pose.position.x = latest_poses_[i].position.x;
                marker.pose.position.y = latest_poses_[i].position.y;
                marker.pose.position.z = latest_poses_[i].position.z;
                marker.pose.orientation = latest_poses_[i].orientation;

                marker.scale.x = marker_length_;
                marker.scale.y = marker_length_;
                marker.scale.z = 0.01;  // thin cube for flat marker visualization

                marker.color.r = 1.0f;
                marker.color.g = 1.0f;
                marker.color.b = 1.0f;
                marker.color.a = 1.0f;

                // Set lifetime to ensure stale markers are cleaned up
                marker.lifetime = rclcpp::Duration(2, 0);

                marker_array.markers.push_back(marker);
            }
        }
        marker_array_pub_->publish(marker_array);

        // Publish detections message if enabled
        if (should_publish_output_ && detection_pub_)
        {
            perseus_interfaces::msg::ObjectDetections detection_msg;
            detection_msg.stamp = header.stamp;
            detection_msg.frame_id = tf_output_frame_;
            {
                std::lock_guard<std::mutex> lock(detections_mutex_);
                detection_msg.ids = latest_ids_;
                detection_msg.poses = latest_poses_;
                detection_msg.regions_of_interest = latest_regions_of_interest_;
            }
            detection_pub_->publish(detection_msg);
        }
    }

    void ArucoDetector::transform_and_publish_marker(const std_msgs::msg::Header& header, int marker_id,
                                                     const sensor_msgs::msg::RegionOfInterest& region_of_interest,
                                                     const cv::Vec3d& rvec, const cv::Vec3d& tvec)
    {
        try
        {
            geometry_msgs::msg::PoseStamped marker_pose_camera;
            marker_pose_camera.header.stamp = header.stamp;
            marker_pose_camera.header.frame_id = camera_frame_;

            marker_pose_camera.pose.position.x = tvec[0];
            marker_pose_camera.pose.position.y = tvec[1];
            marker_pose_camera.pose.position.z = tvec[2];

            // Convert rotation vector to quaternion via tf2::Matrix3x3
            cv::Mat rotation_matrix;
            cv::Rodrigues(rvec, rotation_matrix);

            tf2::Matrix3x3 tf2_rot(
                rotation_matrix.at<double>(0, 0), rotation_matrix.at<double>(0, 1), rotation_matrix.at<double>(0, 2),
                rotation_matrix.at<double>(1, 0), rotation_matrix.at<double>(1, 1), rotation_matrix.at<double>(1, 2),
                rotation_matrix.at<double>(2, 0), rotation_matrix.at<double>(2, 1), rotation_matrix.at<double>(2, 2));

            tf2::Quaternion quat;
            tf2_rot.getRotation(quat);

            marker_pose_camera.pose.orientation.x = quat.x();
            marker_pose_camera.pose.orientation.y = quat.y();
            marker_pose_camera.pose.orientation.z = quat.z();
            marker_pose_camera.pose.orientation.w = quat.w();

            geometry_msgs::msg::PoseStamped marker_pose_out;
            tf_buffer_->transform(marker_pose_camera, marker_pose_out, tf_output_frame_);

            // Cache this detection for service requests
            {
                std::lock_guard<std::mutex> lock(detections_mutex_);
                latest_ids_.push_back(marker_id);
                latest_poses_.push_back(marker_pose_out.pose);
                latest_regions_of_interest_.push_back(region_of_interest);
            }

            geometry_msgs::msg::TransformStamped transform;
            transform.header.stamp = header.stamp;
            transform.header.frame_id = tf_output_frame_;
            transform.child_frame_id = "aruco_marker_" + std::to_string(marker_id);

            transform.transform.translation.x = marker_pose_out.pose.position.x;
            transform.transform.translation.y = marker_pose_out.pose.position.y;
            transform.transform.translation.z = marker_pose_out.pose.position.z;

            transform.transform.rotation = marker_pose_out.pose.orientation;

            if (should_publish_tf_)
            {
                tf_broadcaster_->sendTransform(transform);
            }

            RCLCPP_DEBUG(this->get_logger(),
                         "ArUco %d in %s: x=%.2f, y=%.2f, z=%.2f",
                         marker_id, tf_output_frame_.c_str(),
                         marker_pose_out.pose.position.x,
                         marker_pose_out.pose.position.y,
                         marker_pose_out.pose.position.z);
        }
        catch (tf2::TransformException& ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not transform marker pose: %s", ex.what());
        }
    }

    void ArucoDetector::handle_request(const std::shared_ptr<DetectObjects::Request> request,
                                       std::shared_ptr<DetectObjects::Response> response)
    {
        // Get data snapshot while holding lock, then release before I/O
        cv::Mat frame_to_save;
        std::vector<std::pair<int, cv::Point3d>> marker_coords_copy;
        std::vector<int> ids_copy;
        size_t detection_count = 0;
        {
            std::lock_guard<std::mutex> lock(detections_mutex_);

            response->stamp = latest_timestamp_;
            response->frame_id = tf_output_frame_;
            response->ids = latest_ids_;
            response->poses = latest_poses_;
            response->message = latest_ids_.empty()
                                    ? "No ArUco detections are currently cached."
                                    : "Returned cached ArUco detections.";
            detection_count = latest_ids_.size();

            // Copy data needed for image processing
            if (request->capture_image)
            {
                frame_to_save = latest_frame_.clone();
                marker_coords_copy = latest_marker_coords_;
                ids_copy = latest_ids_;
            }
        }

        // Handle image capture if requested (outside of lock)
        if (request->capture_image)
        {
            if (frame_to_save.empty())
            {
                RCLCPP_WARN(this->get_logger(), "Capture requested but no frame available");
            }
            else
            {
                try
                {
                    // Create directory if it doesn't exist
                    std::error_code ec;
                    std::filesystem::create_directories(request->img_save_path, ec);
                    if (ec)
                    {
                        RCLCPP_WARN(this->get_logger(), "Failed to create directory: %s", request->img_save_path.c_str());
                    }
                    else
                    {
                        // Add timestamp to image (draw directly on frame_to_save, already a copy)
                        auto now = std::chrono::system_clock::now();
                        auto time_t = std::chrono::system_clock::to_time_t(now);
                        auto epoch_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                            now.time_since_epoch())
                                            .count();
                        struct tm* timeinfo = std::localtime(&time_t);
                        char timestamp_str[100];
                        std::strftime(timestamp_str, sizeof(timestamp_str), "%Y-%m-%d %H:%M:%S", timeinfo);

                        cv::putText(frame_to_save, std::string("Time: ") + timestamp_str,
                                    cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                                    cv::Scalar(0, 255, 0), 2);

                        // Add marker coordinates to image
                        if (!marker_coords_copy.empty())
                        {
                            int y_offset = 70;
                            cv::putText(frame_to_save, "Marker Coordinates (XYZ):",
                                        cv::Point(10, y_offset), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                                        cv::Scalar(0, 255, 255), 1);

                            for (const auto& mc : marker_coords_copy)
                            {
                                y_offset += 25;
                                char coord_str[150];
                                std::snprintf(coord_str, sizeof(coord_str),
                                              "ID %d: X=%.3f, Y=%.3f, Z=%.3f",
                                              mc.first, mc.second.x, mc.second.y, mc.second.z);

                                cv::putText(frame_to_save, std::string(coord_str),
                                            cv::Point(10, y_offset), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                                            cv::Scalar(255, 255, 0), 1);
                            }
                        }

                        // Generate filename with marker IDs and timestamp to prevent overwrites
                        std::string ids_str;
                        for (size_t i = 0; i < ids_copy.size(); ++i)
                        {
                            if (i > 0)
                                ids_str += "_";
                            ids_str += std::to_string(ids_copy[i]);
                        }
                        if (ids_str.empty())
                            ids_str = "no_markers";

                        std::string filename = request->img_save_path + "/aruco_" + ids_str +
                                               "_" + std::to_string(epoch_ms) + ".png";

                        if (cv::imwrite(filename, frame_to_save))
                        {
                            RCLCPP_INFO(this->get_logger(), "Captured annotated image saved to: %s", filename.c_str());
                        }
                        else
                        {
                            RCLCPP_ERROR(this->get_logger(), "Failed to write image to: %s", filename.c_str());
                        }
                    }
                }
                catch (const std::exception& e)
                {
                    RCLCPP_ERROR(this->get_logger(), "Exception during image capture: %s", e.what());
                }
            }
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

    void ArucoDetector::camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(camera_matrix_mutex_);

        // Extract camera matrix K (3x3)
        camera_matrix_ = cv::Mat(3, 3, CV_64F);
        camera_matrix_.at<double>(0, 0) = msg->k[0];  // fx
        camera_matrix_.at<double>(0, 1) = msg->k[1];  // skew (usually 0)
        camera_matrix_.at<double>(0, 2) = msg->k[2];  // cx
        camera_matrix_.at<double>(1, 0) = msg->k[3];  // 0
        camera_matrix_.at<double>(1, 1) = msg->k[4];  // fy
        camera_matrix_.at<double>(1, 2) = msg->k[5];  // cy
        camera_matrix_.at<double>(2, 0) = msg->k[6];  // 0
        camera_matrix_.at<double>(2, 1) = msg->k[7];  // 0
        camera_matrix_.at<double>(2, 2) = msg->k[8];  // 1

        // Extract distortion coefficients
        dist_coeffs_ = cv::Mat(msg->d.size(), 1, CV_64F);
        for (size_t i = 0; i < msg->d.size(); ++i)
        {
            dist_coeffs_.at<double>(i, 0) = msg->d[i];
        }

        RCLCPP_DEBUG(this->get_logger(), "Updated camera calibration from camera_info topic");
    }

}  // namespace perseus_vision
