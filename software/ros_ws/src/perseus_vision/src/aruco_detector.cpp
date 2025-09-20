#include "perseus_vision/aruco_detector.hpp"

ArucoDetector::ArucoDetector()
    : Node("aruco_detector")
{
    // Declare and load parameters
    marker_length_ = this->declare_parameter<double>("marker_length", 0.35);
    axis_length_ = this->declare_parameter<double>("axis_length", 0.03);

    camera_frame_ = this->declare_parameter<std::string>("camera_frame", "camera_link_optical");
    tf_output_frame_ = this->declare_parameter<std::string>("tf_output_frame", "odom");

    std::vector<double> camera_matrix_param = this->declare_parameter<std::vector<double>>(
        "camera_matrix", {530.4, 0.0, 320.0, 0.0, 530.4, 240.0, 0.0, 0.0, 1.0});
    std::vector<double> dist_coeffs_param = this->declare_parameter<std::vector<double>>(
        "dist_coeffs", {0, 0, 0, 0, 0});

    // Convert parameters to OpenCV matrices
    camera_matrix_ = cv::Mat(3, 3, CV_64F, camera_matrix_param.data()).clone();
    dist_coeffs_ = cv::Mat(dist_coeffs_param).clone();

    // ArUco setup
    dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100);
    detector_ = cv::aruco::ArucoDetector(dictionary_);

    // TF broadcaster and listener
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    // Image subscriber
    sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/rgbd_camera/image_raw", 10,
        std::bind(&ArucoDetector::imageCallback, this, std::placeholders::_1));

    // Processed image publisher
    pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        "/detection/aruco/image", 10);

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

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    detector_.detectMarkers(frame, corners, ids);

    if (!ids.empty())
    {
        std::vector<cv::Vec3d> rvecs, tvecs;
        cv::aruco::estimatePoseSingleMarkers(corners, marker_length_, camera_matrix_, dist_coeffs_, rvecs, tvecs);

        cv::aruco::drawDetectedMarkers(frame, corners, ids);

        for (size_t i = 0; i < ids.size(); ++i)
        {
            cv::drawFrameAxes(frame, camera_matrix_, dist_coeffs_, rvecs[i], tvecs[i], axis_length_);
            transformAndPublishMarker(msg->header, ids[i], rvecs[i], tvecs[i]);
        }
    }

    auto processed_msg = cv_bridge::CvImage(msg->header, "bgr8", frame).toImageMsg();
    pub_->publish(*processed_msg);
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

        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = header.stamp;
        transform.header.frame_id = tf_output_frame_;
        transform.child_frame_id = "aruco_marker_" + std::to_string(marker_id);

        transform.transform.translation.x = marker_pose_out.pose.position.x;
        transform.transform.translation.y = marker_pose_out.pose.position.y;
        transform.transform.translation.z = marker_pose_out.pose.position.z;

        transform.transform.rotation = marker_pose_out.pose.orientation;

        tf_broadcaster_->sendTransform(transform);

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
