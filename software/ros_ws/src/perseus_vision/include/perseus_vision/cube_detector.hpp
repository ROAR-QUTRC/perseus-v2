#include <memory>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "onnxruntime/onnxruntime_cxx_api.h"
#include "opencv2/opencv.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"
#include "vision_msgs/msg/detection2_d.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"
#include "vision_msgs/msg/object_hypothesis_with_pose.hpp"

namespace perseus_vision
{
    static const std::vector<std::string> CLASS_NAMES = {"blue", "red", "green", "yellow"};

    // BGR colours for annotation
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
        static constexpr int _kQosDepth = 10;  // QoS depth for subscriptions and publishers

        void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
        void process_image(const cv::Mat& frame, const std_msgs::msg::Header& header);
        void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

        // ROS IO
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _sub;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _pub;
        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr _camera_info_sub;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _marker_array_pub;

        std::mutex _camera_matrix_mutex;
        cv::Mat _camera_matrix_;
        cv::Mat _dist_coeffs_;

        // Cube detection parameters
        std::string _model_path;
        double _confidence_threshold;
        std::string _camera_topic;
        std::string _camera_info_topic;
        std::string _detection_topic;
    };

}  // namespace perseus_vision