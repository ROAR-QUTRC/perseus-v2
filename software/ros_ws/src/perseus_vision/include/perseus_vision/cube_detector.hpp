#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>

// #include "ament_index_cpp/get_package_share_directory.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "onnxruntime/onnxruntime_cxx_api.h"
#include "opencv2/opencv.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"
namespace perseus_vision
{

    static const std::vector<std::string> CLASS_NAMES = {"blue", "red", "green", "yellow"};

    static const std::vector<cv::Scalar> CLASS_COLOURS = {
        cv::Scalar(255, 100, 0),  // blue
        cv::Scalar(0, 0, 255),    // red
        cv::Scalar(0, 200, 0),    // green
        cv::Scalar(0, 220, 255),  // yellow
    };

    struct Detection
    {
        int class_id;
        float confidence;
        cv::Rect bbox;  // in original image coordinates
    };

    class CubeDetector : public rclcpp::Node
    {
    public:
        CubeDetector();

    private:
        static constexpr int kQosDepth = 10;

        // callbacks
        void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
        void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

        // inference helpers
        std::vector<float> preprocess(const cv::Mat& bgr_image);
        std::vector<Detection> postprocess(const float* data, size_t num_boxes);

        // publishing helpers
        void publish_annotated_image(
            const cv::Mat& image,
            const std::vector<Detection>& detections,
            const std_msgs::msg::Header& header);

        // void publish_detections(
        //     const std::vector<Detection>& detections,
        //     const std_msgs::msg::Header& header);

        // ONNX Runtime
        Ort::Env ort_env_;
        Ort::AllocatorWithDefaultOptions ort_allocator_;
        std::unique_ptr<Ort::Session> ort_session_;
        std::string input_name_;
        std::string output_name_;

        // image size tracking for coordinate scaling
        int orig_h_{0};
        int orig_w_{0};

        // parameters
        std::string model_path_;
        float confidence_threshold_;

        // camera calibration (kept for future use)
        std::mutex camera_matrix_mutex_;
        cv::Mat camera_matrix_;
        cv::Mat dist_coeffs_;

        // subscribers
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;
        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_camera_info_;
        
        // publishers
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_annotated_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_colour_;
    };

}  // namespace perseus_vision