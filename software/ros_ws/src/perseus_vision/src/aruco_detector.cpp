#include "perseus_vision/aruco_detector.hpp"

namespace perseus_vision {

ArucoDetector::ArucoDetector()
    : Node("aruco_detector")
{
    // Declare and load parameters
    marker_length_ = this->declare_parameter<double>("marker_length", 0.35);
    axis_length_ = this->declare_parameter<double>("axis_length", 0.03);
    dictionary_id = this->declare_parameter<int>("dictionary_id", 1);
    camera_frame_ = this->declare_parameter<std::string>("camera_frame", "camera_link_optical");
    tf_output_frame_ = this->declare_parameter<std::string>("tf_output_frame", "odom");
    input_img_ = this->declare_parameter<std::string>("input_img", "/rgbd_camera/image_raw");
    output_img_ = this->declare_parameter<std::string>("output_img", "/detection/aruco/image");
    publish_tf_ = this->declare_parameter<bool>("publish_tf", true);
    publish_img_ = this->declare_parameter<bool>("publish_img", true);
    compressed_io_ = this->declare_parameter<bool>("compressed_io", false);
    publish_output_ = this->declare_parameter<bool>("publish_output", false);
    use_camera_info_ = this->declare_parameter<bool>("use_camera_info", false);
    output_topic_ = this->declare_parameter<std::string>("output_topic", "/detection/aruco/detections");
    camera_info_topic_ = this->declare_parameter<std::string>("camera_info_topic", "/camera/camera_info");

    std::vector<double> camera_matrix_param = this->declare_parameter<std::vector<double>>(
        "camera_matrix", {530.4, 0.0, 320.0, 0.0, 530.4, 240.0, 0.0, 0.0, 1.0});
    std::vector<double> dist_coeffs_param = this->declare_parameter<std::vector<double>>(
        "distortion_coefficients", {0, 0, 0, 0, 0});

    // Convert parameters to OpenCV matrices (will be overwritten by camera_info if use_camera_info is true)
    if (!use_camera_info_)
    {
        camera_matrix_ = cv::Mat(3, 3, CV_64F, camera_matrix_param.data()).clone();
        dist_coeffs_ = cv::Mat(dist_coeffs_param).clone();
    }

    // ArUco setup
    cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(dictionary_id);
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
    }  pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            output_img_, 10);
    }
    //
    service_ = this->create_service<DetectObjects>(
      "detect_objects",
      std::bind(&ArucoDetector::handle_request, this,
                std::placeholders::_1,
                std::placeholders::_2));
    if (publish_output_) {
        detection_pub_ = this->create_publisher<::perseus_interfaces::msg::ObjectDetections>(
            output_topic_, 10);
    } 
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
        auto processed_msg = cv_bridge::CvImage(msg->header, "bgr8", frame).toImageMsg();
        pub_->publish(*processed_msg);
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
        
        std::vector<uchar> buffer;
        std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 90};
        cv::imencode(".jpg", frame, buffer, params);
        compressed_msg.data = buffer;
        
        compressed_pub_->publish(compressed_msg);
    }
}

void ArucoDetector::processImage(const cv::Mat& frame, const std_msgs::msg::Header& header)
{
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    detector_.detectMarkers(frame, corners, ids);

    // Clear previous detections and update timestamp for service requests
    {
        std::lock_guard<std::mutex> lock(detections_mutex_);
        latest_ids_.clear();
        latest_poses_.clear();
        latest_timestamp_ = header.stamp;
        has_detections_ = true;
    }

    if (!ids.empty())
    {
        std::vector<cv::Vec3d> rvecs, tvecs;
        cv::aruco::estimatePoseSingleMarkers(corners, marker_length_, camera_matrix_, dist_coeffs_, rvecs, tvecs);

        cv::aruco::drawDetectedMarkers(const_cast<cv::Mat&>(frame), corners, ids);

        for (size_t i = 0; i < ids.size(); ++i)
        {
            cv::drawFrameAxes(const_cast<cv::Mat&>(frame), camera_matrix_, dist_coeffs_, rvecs[i], tvecs[i], axis_length_);
            transformAndPublishMarker(header, ids[i], rvecs[i], tvecs[i]);
        }
    }

    // Publish detections message if enabled
    if (publish_output_ && detection_pub_)
    {
        ::perseus_interfaces::msg::ObjectDetections detection_msg;
        {
            std::lock_guard<std::mutex> lock(detections_mutex_);
            detection_msg.stamp = latest_timestamp_;
            detection_msg.frame_id = tf_output_frame_;
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
        geometry_msgs::msg::PoseStamped  marker_pose_camera;
        marker_pose_camera.header.stamp = header.stamp;
        marker_pose_camera.header.frame_id = camera_frame_;

        // OpenCV to ROS coordinate adjustment
        marker_pose_camera.pose.position.x = tvec[2];
        marker_pose_camera.pose.position.y = -tvec[0];
        marker_pose_camera.pose.position.z = -tvec[1];

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

        if (publish_tf_) {
            tf_broadcaster_->sendTransform(transform);
        }

        RCLCPP_INFO(this->get_logger(),
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
    (void)request;  // Suppress unused parameter warning
    
    std::lock_guard<std::mutex> lock(detections_mutex_);
    
    response->stamp = latest_timestamp_;
    response->frame_id = tf_output_frame_;
    response->ids = latest_ids_;
    response->poses = latest_poses_;
    
    if (!latest_ids_.empty()) {
        RCLCPP_INFO(this->get_logger(), "Service request: returning %zu detections", latest_ids_.size());
    } else {
        RCLCPP_INFO(this->get_logger(), "Service request: no detections available");
    }
}

void ArucoDetector::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
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