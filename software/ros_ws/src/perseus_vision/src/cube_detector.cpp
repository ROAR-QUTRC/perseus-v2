#include "perseus_vision/cube_detector.hpp"

#include <chrono>
#include <cstdio>
#include <ctime>
#include <filesystem>

namespace perseus_vision
{

    CubeDetector::CubeDetector()
        : Node("cube_detector")
    {
        _sub = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/camera/color/image_raw", _kQosDepth,
            std::bind(&CubeDetector::image_callback, this, std::placeholders::_1));

        _camera_info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/camera/camera/color/camera_info", _kQosDepth,
            std::bind(&CubeDetector::camera_info_callback, this, std::placeholders::_1));

        _marker_array_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/perseus_vision/cube/markers", _kQosDepth);
    }

    void CubeDetector::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        process_image(cv_ptr->image, msg->header);
    }
    void CubeDetector::camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        // Ensure thread safety when updating camera parameters by locking the mutex
        std::lock_guard<std::mutex> lock(_camera_matrix_mutex);

        // Extract camera matrix K (3x3)
        _camera_matrix_ = cv::Mat(3, 3, CV_64F);
        _camera_matrix_.at<double>(0, 0) = msg->k[0];  // fx
        _camera_matrix_.at<double>(0, 1) = msg->k[1];  // skew (usually 0)
        _camera_matrix_.at<double>(0, 2) = msg->k[2];  // cx
        _camera_matrix_.at<double>(1, 0) = msg->k[3];  // 0
        _camera_matrix_.at<double>(1, 1) = msg->k[4];  // fy
        _camera_matrix_.at<double>(1, 2) = msg->k[5];  // cy
        _camera_matrix_.at<double>(2, 0) = msg->k[6];  // 0
        _camera_matrix_.at<double>(2, 1) = msg->k[7];  // 0
        _camera_matrix_.at<double>(2, 2) = msg->k[8];  // 1

        // Extract distortion coefficients
        _dist_coeffs_ = cv::Mat(msg->d.size(), 1, CV_64F);
        for (size_t i = 0; i < msg->d.size(); ++i)
        {
            _dist_coeffs_.at<double>(i, 0) = msg->d[i];
        }

        RCLCPP_DEBUG(this->get_logger(), "Updated camera calibration from camera_info topic");
    }
}  // namespace perseus_vision