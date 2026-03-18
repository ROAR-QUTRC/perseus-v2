#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "onnxruntime/onnxruntime_cxx_api.h"
#include "opencv2/opencv.hpp"
#include "perseus_interfaces/msg/object_detections.hpp"
#include "perseus_interfaces/srv/detect_objects.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"
namespace perseus_vision
{

    inline const std::vector<std::string> CLASS_NAMES = {"blue", "green", "red", "white"};

    inline const std::vector<cv::Scalar> CLASS_COLOURS = {
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
        using DetectObjects = perseus_interfaces::srv::DetectObjects;
        CubeDetector();

    private:
        static constexpr int kQosDepth = 10;

        // callbacks
        void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
        void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
        void depth_image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
        void depth_camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

        // inference helpers
        std::vector<float> preprocess(const cv::Mat& bgr_image);
        std::vector<Detection> postprocess(const float* data, size_t num_boxes);

        // publishing helpers
        void publish_annotated_image(
            const cv::Mat& image,
            const std::vector<Detection>& detections,
            const std_msgs::msg::Header& header);
        void publish_depth_markers(
            const std::vector<Detection>& detections,
            const std_msgs::msg::Header& header);
        bool estimate_cube_pose_from_depth(
            const Detection& detection,
            geometry_msgs::msg::Pose& pose_out,
            double& distance_m_out) const;
        bool read_depth_meters_at_pixel(
            const sensor_msgs::msg::Image& depth_msg,
            int u,
            int v,
            double& depth_m_out) const;
        geometry_msgs::msg::Quaternion quaternion_from_yaw_pitch(double yaw, double pitch) const;
        bool transform_pose_to_output_frame(
            const geometry_msgs::msg::Pose& input_pose,
            const std::string& source_frame,
            const builtin_interfaces::msg::Time& stamp,
            geometry_msgs::msg::Pose& output_pose) const;
        void run_inference_pipeline(
            const cv::Mat& bgr_image,
            const std_msgs::msg::Header& header);
        void handle_detect_objects_request(
            const std::shared_ptr<DetectObjects::Request> request,
            std::shared_ptr<DetectObjects::Response> response);

        // ONNX Runtime
        Ort::Env _ort_env;
        Ort::AllocatorWithDefaultOptions _ort_allocator;
        std::unique_ptr<Ort::Session> _ort_session;
        std::string _input_name;
        std::string _output_name;

        // image size tracking for coordinate scaling
        int _orig_h{0};
        int _orig_w{0};
        int _pad_x{0};
        int _pad_y{0};
        float _letterbox_scale{1.0f};
        const int _num_classes = static_cast<int>(CLASS_NAMES.size());
        // parameters
        std::string _model_path;
        std::string _depth_estimation_mode;
        std::string _depth_topic;
        std::string _depth_info_topic;
        std::string _tf_output_frame;
        std::string _output_img_topic;
        std::string _output_detections_topic;
        std::string _output_markers_topic;

        float _confidence_threshold;
        std::atomic_bool _always_on{true};
        bool _should_use_cuda;
        bool _publish_annotated_image;
        double _depth_unit_scale{0.001};
        double _depth_max_range_m{5.0};
        double _depth_min_range_m{0.1};
        double _processing_frequency_hz{0.0};
        int64_t _last_inference_time_ns{0};
        int _intra_op_num_threads;
        int _inter_op_num_threads;
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr _param_callback_handle;
        float _nms_iou_threshold{0.45f};
        // camera calibration (kept for future use)
        std::mutex _camera_matrix_mutex;
        cv::Mat _camera_matrix;
        cv::Mat _dist_coeffs;
        mutable std::mutex _depth_mutex;
        sensor_msgs::msg::Image::SharedPtr _latest_depth_image;
        bool _has_depth_intrinsics{false};
        double _depth_fx{0.0};
        double _depth_fy{0.0};
        double _depth_cx{0.0};
        double _depth_cy{0.0};

        // subscribers
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _sub_image;
        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr _sub_camera_info;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _sub_depth_image;
        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr _sub_depth_camera_info;

        // publishers
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _pub_annotated;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _pub_cube_markers;
        rclcpp::Publisher<perseus_interfaces::msg::ObjectDetections>::SharedPtr _pub_cube_detections;
        rclcpp::Service<DetectObjects>::SharedPtr _srv_detect_objects;
        std::unique_ptr<tf2_ros::Buffer> _tf_buffer;
        std::shared_ptr<tf2_ros::TransformListener> _tf_listener;
        mutable std::mutex _latest_image_mutex;
        cv::Mat _latest_bgr_frame;
        std_msgs::msg::Header _latest_bgr_header;
        bool _has_latest_bgr_frame{false};
        mutable std::mutex _inference_mutex;
        mutable std::mutex _detections_mutex;
        builtin_interfaces::msg::Time _latest_detections_stamp;
        std::string _latest_detections_frame_id;
        std::vector<int32_t> _latest_detection_ids;
        std::vector<geometry_msgs::msg::Pose> _latest_detection_poses;
        std::vector<cv::Rect> _latest_detection_bboxes;
        cv::Mat _latest_annotated_frame;
    };

}  // namespace perseus_vision
