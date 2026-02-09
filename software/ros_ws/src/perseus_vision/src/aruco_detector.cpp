#include "perseus_vision/aruco_detector.hpp"

#include <chrono>
#include <cstdio>
#include <ctime>
#include <filesystem>

namespace perseus_vision
{

    ArucoDetector::ArucoDetector()
        : Node("aruco_detector")
    {
        // Declare and load parameters
        marker_length_ = this->declare_parameter<double>("marker_length", 0.35);
        axis_length_ = this->declare_parameter<double>("axis_length", 0.03);
        dictionary_id_ = this->declare_parameter<int>("dictionary_id", 1);
        camera_frame_ = this->declare_parameter<std::string>("camera_frame", "camera_link_optical");
        tf_output_frame_ = this->declare_parameter<std::string>("tf_output_frame", "odom");
        input_img_ = this->declare_parameter<std::string>("input_img", "/camera/camera/color/image_raw");
        output_img_ = this->declare_parameter<std::string>("output_img", "/perseus_vision/aruco/image");
        publish_tf_ = this->declare_parameter<bool>("publish_tf", true);
        publish_img_ = this->declare_parameter<bool>("publish_img", true);
        compressed_io_ = this->declare_parameter<bool>("compressed_io", false);
        publish_output_ = this->declare_parameter<bool>("publish_output", false);
        use_camera_info_ = this->declare_parameter<bool>("use_camera_info", false);
        output_topic_ = this->declare_parameter<std::string>("output_topic", "/perseus_vision/aruco/detections");
        camera_info_topic_ = this->declare_parameter<std::string>("camera_info_topic", "/camera/camera/color/camera_info");
        min_bounding_box_area_ = this->declare_parameter<double>("min_bounding_box_area", 100.0);

        std::vector<double> camera_matrix_param = this->declare_parameter<std::vector<double>>(
            "camera_matrix", {530.4, 0.0, 320.0, 0.0, 530.4, 240.0, 0.0, 0.0, 1.0});
        std::vector<double> dist_coeffs_param = this->declare_parameter<std::vector<double>>(
            "distortion_coefficients", {0.0, 0.0, 0.0, 0.0, 0.0});

        // Convert parameters to OpenCV matrices (will be overwritten by camera_info if use_camera_info is true)
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
        if (compressed_io_)
        {
            compressed_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
                input_img_ + "/compressed", 10,
                std::bind(&ArucoDetector::compressedImageCallback, this, std::placeholders::_1));
            compressed_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
                output_img_ + "/compressed", 10);
        }
        else
        {
            sub_ = this->create_subscription<sensor_msgs::msg::Image>(
                input_img_, 10,
                std::bind(&ArucoDetector::imageCallback, this, std::placeholders::_1));

            // Camera info subscriber if enabled
            if (use_camera_info_)
            {
                camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
                    camera_info_topic_,
                    rclcpp::SensorDataQoS(),
                    std::bind(&ArucoDetector::cameraInfoCallback, this, std::placeholders::_1));

                RCLCPP_INFO(this->get_logger(), "Subscribing to camera_info from topic: %s", camera_info_topic_.c_str());
            }
            pub_ = this->create_publisher<sensor_msgs::msg::Image>(
                output_img_, 10);
        }
        //
        service_ = this->create_service<DetectObjects>(
            "detect_objects",
            std::bind(&ArucoDetector::handle_request, this,
                      std::placeholders::_1,
                      std::placeholders::_2));
        if (publish_output_)
        {
            detection_pub_ = this->create_publisher<perseus_interfaces::msg::ObjectDetections>(
                output_topic_, 10);
        }

        // Create marker array publisher for visualization
        marker_array_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/detection/aruco/markers", 10);

        RCLCPP_INFO(this->get_logger(), "Perseus' ArucoDetector node started.");
    }

    void ArucoDetector::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
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

        processImage(frame, msg->header);

        if (publish_img_)
        {
            // Get the latest annotated frame
            {
                std::lock_guard<std::mutex> lock(detections_mutex_);
                auto processed_msg = cv_bridge::CvImage(msg->header, "bgr8", latest_frame_).toImageMsg();
                pub_->publish(*processed_msg);
            }
        }
    }

    void ArucoDetector::compressedImageCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
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

        processImage(frame, msg->header);

        if (publish_img_)
        {
            sensor_msgs::msg::CompressedImage compressed_msg;
            compressed_msg.header = msg->header;
            compressed_msg.format = "jpeg";

            // Get the latest annotated frame
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

    void ArucoDetector::processImage(const cv::Mat& frame, const std_msgs::msg::Header& header)
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
            latest_timestamp_ = header.stamp;
            has_detections_ = true;
        }

        // Store marker info for corner display
        std::vector<std::pair<int, cv::Point3d>> marker_coords;  // marker_id, position

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
                cv::aruco::estimatePoseSingleMarkers(corners, marker_length_, local_camera_matrix, local_dist_coeffs, rvecs, tvecs);

                cv::aruco::drawDetectedMarkers(annotated_frame, corners, ids);

                for (size_t i = 0; i < ids.size(); ++i)
                {
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

                    transformAndPublishMarker(header, ids[i], rvecs[i], tvecs[i]);
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
            latest_frame_ = annotated_frame.clone();
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

                // Set position from pose
                marker.pose.position.x = latest_poses_[i].position.x;
                marker.pose.position.y = latest_poses_[i].position.y;
                marker.pose.position.z = latest_poses_[i].position.z;

                // Set orientation from pose
                marker.pose.orientation = latest_poses_[i].orientation;

                // Set scale (cube size)
                marker.scale.x = marker_length_;
                marker.scale.y = marker_length_;
                marker.scale.z = 0.01;  // thin cube

                // Set color (RGBA)
                marker.color.r = 1.0f;
                marker.color.g = 1.0f;
                marker.color.b = 1.0f;
                marker.color.a = 1.f;

                // Set lifetime to ensure stale markers are cleaned up
                marker.lifetime = rclcpp::Duration(2, 0);  // 2 seconds

                marker_array.markers.push_back(marker);
            }
        }
        marker_array_pub_->publish(marker_array);

        // Publish detections message if enabled
        if (publish_output_ && detection_pub_)
        {
            perseus_interfaces::msg::ObjectDetections detection_msg;
            detection_msg.stamp = header.stamp;
            detection_msg.frame_id = tf_output_frame_;
            {
                std::lock_guard<std::mutex> lock(detections_mutex_);
                detection_msg.ids = latest_ids_;
                detection_msg.poses = latest_poses_;
            }
            detection_pub_->publish(detection_msg);
        }
    }

    void ArucoDetector::transformAndPublishMarker(const std_msgs::msg::Header& header, int marker_id,
                                                  const cv::Vec3d& rvec, const cv::Vec3d& tvec)
    {
        try
        {
            geometry_msgs::msg::PoseStamped marker_pose_camera;
            marker_pose_camera.header.stamp = header.stamp;
            marker_pose_camera.header.frame_id = camera_frame_;

            // OpenCV to ROS coordinate adjustment
            marker_pose_camera.pose.position.x = tvec[0];
            marker_pose_camera.pose.position.y = tvec[1];
            marker_pose_camera.pose.position.z = tvec[2];

            cv::Mat rotation_matrix;
            cv::Rodrigues(rvec, rotation_matrix);
            tf2::Quaternion quat = rotationMatrixToQuaternion(rotation_matrix);

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
            }

            geometry_msgs::msg::TransformStamped transform;
            transform.header.stamp = header.stamp;
            transform.header.frame_id = tf_output_frame_;
            transform.child_frame_id = "aruco_marker_" + std::to_string(marker_id);

            transform.transform.translation.x = marker_pose_out.pose.position.x;
            transform.transform.translation.y = marker_pose_out.pose.position.y;
            transform.transform.translation.z = marker_pose_out.pose.position.z;

            transform.transform.rotation = marker_pose_out.pose.orientation;

            if (publish_tf_)
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

    tf2::Quaternion ArucoDetector::rotationMatrixToQuaternion(const cv::Mat& rotation_matrix)
    {
        // Extract rotation matrix elements
        double r11 = rotation_matrix.at<double>(0, 0);
        double r12 = rotation_matrix.at<double>(0, 1);
        double r13 = rotation_matrix.at<double>(0, 2);
        double r21 = rotation_matrix.at<double>(1, 0);
        double r22 = rotation_matrix.at<double>(1, 1);
        double r23 = rotation_matrix.at<double>(1, 2);
        double r31 = rotation_matrix.at<double>(2, 0);
        double r32 = rotation_matrix.at<double>(2, 1);
        double r33 = rotation_matrix.at<double>(2, 2);

        tf2::Quaternion quat;
        double trace = r11 + r22 + r33;
        if (trace > 0)
        {
            double s = sqrt(trace + 1.0) * 2;
            quat.setW(0.25 * s);
            quat.setX((r32 - r23) / s);
            quat.setY((r13 - r31) / s);
            quat.setZ((r21 - r12) / s);
        }
        else if ((r11 > r22) && (r11 > r33))
        {
            double s = sqrt(1.0 + r11 - r22 - r33) * 2;
            quat.setW((r32 - r23) / s);
            quat.setX(0.25 * s);
            quat.setY((r12 + r21) / s);
            quat.setZ((r13 + r31) / s);
        }
        else if (r22 > r33)
        {
            double s = sqrt(1.0 + r22 - r11 - r33) * 2;
            quat.setW((r13 - r31) / s);
            quat.setX((r12 + r21) / s);
            quat.setY(0.25 * s);
            quat.setZ((r23 + r32) / s);
        }
        else
        {
            double s = sqrt(1.0 + r33 - r11 - r22) * 2;
            quat.setW((r21 - r12) / s);
            quat.setX((r13 + r31) / s);
            quat.setY((r23 + r32) / s);
            quat.setZ(0.25 * s);
        }
        return quat;
    }

    void ArucoDetector::handle_request(const std::shared_ptr<DetectObjects::Request> request,
                                       std::shared_ptr<DetectObjects::Response> response)
    {
        // Get data snapshot while holding lock, then release before I/O
        cv::Mat frame_to_save;
        std::vector<std::pair<int, cv::Point3d>> marker_coords_copy;
        std::vector<int> ids_copy;
        {
            std::lock_guard<std::mutex> lock(detections_mutex_);

            response->stamp = latest_timestamp_;
            response->frame_id = tf_output_frame_;
            response->ids = latest_ids_;
            response->poses = latest_poses_;

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
                        // Create annotated copy for output
                        cv::Mat output_frame = frame_to_save.clone();

                        // Add timestamp to image
                        {
                            auto now = std::chrono::system_clock::now();
                            auto time_t = std::chrono::system_clock::to_time_t(now);
                            struct tm* timeinfo = std::localtime(&time_t);
                            char timestamp_str[100];
                            std::strftime(timestamp_str, sizeof(timestamp_str), "%Y-%m-%d %H:%M:%S", timeinfo);

                            cv::putText(output_frame, std::string("Time: ") + timestamp_str,
                                        cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                                        cv::Scalar(0, 255, 0), 2);
                        }

                        // Add marker coordinates to image
                        if (!marker_coords_copy.empty())
                        {
                            int y_offset = 70;
                            cv::putText(output_frame, "Marker Coordinates (XYZ):",
                                        cv::Point(10, y_offset), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                                        cv::Scalar(0, 255, 255), 1);

                            for (const auto& mc : marker_coords_copy)
                            {
                                y_offset += 25;
                                char coord_str[150];
                                std::snprintf(coord_str, sizeof(coord_str),
                                              "ID %d: X=%.3f, Y=%.3f, Z=%.3f",
                                              mc.first, mc.second.x, mc.second.y, mc.second.z);

                                cv::putText(output_frame, std::string(coord_str),
                                            cv::Point(10, y_offset), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                                            cv::Scalar(255, 255, 0), 1);
                            }
                        }

                        // Generate filename with marker IDs
                        std::string ids_str;
                        for (size_t i = 0; i < ids_copy.size(); ++i)
                        {
                            if (i > 0)
                                ids_str += "_";
                            ids_str += std::to_string(ids_copy[i]);
                        }
                        if (ids_str.empty())
                            ids_str = "no_markers";

                        std::string filename = request->img_save_path + "/aruco_" + ids_str + ".png";

                        // Save the annotated frame
                        if (cv::imwrite(filename, output_frame))
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

        if (!latest_ids_.empty())
        {
            RCLCPP_INFO(this->get_logger(), "Service request: returning %zu detections", latest_ids_.size());
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Service request: no detections available");
        }
    }

    void ArucoDetector::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(camera_matrix_mutex_);

        // Extract camera matrix K (3x3)
        // Camera matrix is stored as [fx, 0, cx, 0, fy, cy, 0, 0, 1]
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
        // Typically [k1, k2, p1, p2, k3] but can have more
        dist_coeffs_ = cv::Mat(msg->d.size(), 1, CV_64F);
        for (size_t i = 0; i < msg->d.size(); ++i)
        {
            dist_coeffs_.at<double>(i, 0) = msg->d[i];
        }

        RCLCPP_DEBUG(this->get_logger(), "Updated camera calibration from camera_info topic");
    }

}  // namespace perseus_vision
