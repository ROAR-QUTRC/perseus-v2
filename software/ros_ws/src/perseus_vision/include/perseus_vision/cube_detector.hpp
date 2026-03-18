#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "onnxruntime/onnxruntime_cxx_api.h"
#include "opencv2/opencv.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"
namespace perseus_vision
{

    static const std::vector<std::string> CLASS_NAMES = {"blue", "green", "red", "white"};

    static const std::vector<cv::Scalar> CLASS_COLOURS = {
        cv::Scalar(255, 100, 0),    // blue
        cv::Scalar(0, 200, 0),      // green
        cv::Scalar(0, 0, 255),      // red
        cv::Scalar(255, 255, 255),  // white
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
        Ort::Env _ort_env;
        Ort::AllocatorWithDefaultOptions _ort_allocator;
        std::unique_ptr<Ort::Session> _ort_session;
        std::string _input_name;
        std::string _output_name;

        // image size tracking for coordinate scaling
        int _orig_h{0};
        int _orig_w{0};

        // parameters
        std::string _model_path;
        float _confidence_threshold;
        std::atomic_bool _always_on{true};
        bool _should_use_cuda;
        bool _publish_annotated_image;
        int _intra_op_num_threads;
        int _inter_op_num_threads;
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr _param_callback_handle;
        // camera calibration (kept for future use)
        std::mutex _camera_matrix_mutex;
        cv::Mat _camera_matrix;
        cv::Mat _dist_coeffs;

        // subscribers
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _sub_image;
        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr _sub_camera_info;

        // publishers
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _pub_annotated;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _pub_colour;
    };

}  // namespace perseus_vision
