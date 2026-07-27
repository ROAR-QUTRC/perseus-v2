#pragma once

/// @file cube_detector.hpp
/// @brief Cube detection and depth-based pose estimation ROS 2 node.

#include <onnxruntime/onnxruntime_cxx_api.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <array>
#include <atomic>
#include <builtin_interfaces/msg/time.hpp>
#include <cstddef>
#include <cstdint>
#include <cv_bridge/cv_bridge.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <memory>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/region_of_interest.hpp>
#include <std_msgs/msg/header.hpp>
#include <string>
#include <vector>
#include <visualization_msgs/msg/marker_array.hpp>

#include "perseus_interfaces/msg/detection_array.hpp"
#include "perseus_interfaces/srv/detect_objects.hpp"

namespace perseus_vision
{
    /// @brief Number of cube classes the detection model distinguishes.
    inline constexpr std::size_t NUM_CLASSES = 4;

    /// @brief Human-readable name of each cube class, indexed by class ID.
    inline const std::array<std::string, NUM_CLASSES> CLASS_NAMES = {"blue", "green", "red", "white"};

    /// @brief Annotation colour of each cube class in BGR order, indexed by class ID.
    inline const std::array<cv::Scalar, NUM_CLASSES> CLASS_COLORS = {
        cv::Scalar(255, 100, 0),    // blue
        cv::Scalar(0, 200, 0),      // green
        cv::Scalar(0, 0, 255),      // red
        cv::Scalar(255, 255, 255),  // white
    };

    /// @brief One cube found by the detection model in a single image.
    struct detection_t
    {
        /// @brief Index into CLASS_NAMES and CLASS_COLORS.
        int32_t class_id{0};
        /// @brief Model confidence in the range [0, 1].
        float confidence{0.0f};
        /// @brief Bounding box in original (not letterboxed) image coordinates.
        cv::Rect bounding_box{};
    };

    /// @brief ROS 2 node for detecting coloured cubes and estimating their poses.
    ///
    /// Subscribes to RGB images, runs ONNX-based object detection to find cubes,
    /// and can use aligned depth data to estimate 3D poses for publishing as
    /// detection messages and rviz markers. Also provides a service interface for
    /// returning the latest cached detections and optionally saving an annotated image.
    class CubeDetector : public rclcpp::Node
    {
    public:
        using DetectObjects = perseus_interfaces::srv::DetectObjects;

        /// @brief Constructs the node, declaring parameters and setting up
        ///        subscriptions, publishers, inference resources, and the detection service.
        /// @param options Node options, supplied by the component container or by main().
        explicit CubeDetector(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    private:
        /// @brief Default minimum model confidence for a detection to be reported.
        static constexpr float DEFAULT_CONFIDENCE_THRESHOLD = 0.3f;
        /// @brief Default intersection-over-union threshold used by non-maximum suppression.
        static constexpr float DEFAULT_NMS_IOU_THRESHOLD = 0.45f;
        /// @brief Default inference rate limit, in hertz. Zero processes every frame.
        static constexpr double DEFAULT_PROCESSING_FREQUENCY_HZ = 0.0;
        /// @brief Default number of threads ONNX Runtime uses within a single operator.
        static constexpr int DEFAULT_INTRA_OP_NUM_THREADS = 4;
        /// @brief Default number of threads ONNX Runtime uses across operators.
        static constexpr int DEFAULT_INTER_OP_NUM_THREADS = 2;
        /// @brief Default metres represented by one unit of a 16-bit depth image.
        static constexpr double DEFAULT_DEPTH_UNIT_SCALE = 0.001;
        /// @brief Default furthest depth reading accepted as a valid cube distance, in metres.
        static constexpr double DEFAULT_DEPTH_MAX_RANGE_M = 5.0;
        /// @brief Default nearest depth reading accepted as a valid cube distance, in metres.
        static constexpr double DEFAULT_DEPTH_MIN_RANGE_M = 0.1;
        /// @brief Default for whether the node runs inference without any subscribers.
        static constexpr bool DEFAULT_IS_ALWAYS_ON = true;
        /// @brief Default for whether inference runs on the CUDA execution provider.
        static constexpr bool DEFAULT_SHOULD_USE_CUDA = false;

        /// @brief Default pose estimation mode. `none` publishes 2D detections only.
        static inline const std::string DEFAULT_DEPTH_ESTIMATION_MODE = "none";
        /// @brief Default topic that colour images are read from.
        static inline const std::string DEFAULT_CAMERA_TOPIC = "/camera/camera/color/image_raw";
        /// @brief Default topic that colour camera calibration is read from.
        static inline const std::string DEFAULT_CAMERA_INFO_TOPIC = "/camera/camera/color/camera_info";
        /// @brief Default topic that depth images are read from.
        ///
        /// The aligned stream is the default because poses are computed from the depth
        /// intrinsics but published in the colour image's frame. Only an aligned depth
        /// stream shares that frame; a raw one is offset by the camera baseline.
        static inline const std::string DEFAULT_DEPTH_TOPIC =
            "/camera/camera/aligned_depth_to_color/image_raw";
        /// @brief Default topic that depth camera calibration is read from.
        static inline const std::string DEFAULT_DEPTH_INFO_TOPIC =
            "/camera/camera/aligned_depth_to_color/camera_info";
        /// @brief Default topic that detection messages are published on.
        static inline const std::string DEFAULT_OUTPUT_DETECTIONS_TOPIC = "/perseus_vision/cube/detections";
        /// @brief Default topic that rviz visualization markers are published on.
        static inline const std::string DEFAULT_OUTPUT_MARKERS_TOPIC = "/perseus_vision/cube/markers";

        /// @brief Resolves the configured model path against the packaged model directory.
        ///
        /// Handles `~/` expansion, paths missing their `$HOME` prefix, and bare file names,
        /// falling back to the model shipped in the package share directory.
        /// @return The first candidate path that exists, or the configured path if none do.
        std::string _resolve_model_path() const;

        /// @brief Loads the ONNX model and caches its input and output tensor names.
        /// @throws Ort::Exception If the model cannot be loaded.
        void _load_model();

        /// @brief Declares every parameter and copies its value into the matching member.
        /// @param camera_topic_out Receives the resolved colour image topic.
        /// @param camera_info_topic_out Receives the resolved colour camera info topic.
        void _load_parameters(std::string& camera_topic_out, std::string& camera_info_topic_out);

        /// @brief Runs inference on an incoming colour image, subject to the rate limit.
        /// @param msg Incoming colour image message.
        void _image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

        /// @brief Updates the cached colour camera intrinsics.
        /// @param msg Incoming colour camera calibration message.
        void _camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

        /// @brief Caches the most recent depth image for pose estimation.
        /// @param msg Incoming depth image message.
        void _depth_image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

        /// @brief Updates the cached depth camera intrinsics.
        /// @param msg Incoming depth camera calibration message.
        void _depth_camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

        /// @brief Applies a rate limit to inference based on `processing_frequency_hz`.
        /// @return True if this frame should be processed, false if it should be skipped.
        bool _should_run_inference_now();

        /// @brief Letterboxes an image and flattens it into a normalised CHW float tensor.
        /// @param bgr_image Image to prepare for inference.
        /// @return The flattened model input tensor.
        std::vector<float> _preprocess(const cv::Mat& bgr_image);

        /// @brief Converts raw model output into detections in original image coordinates.
        /// @param data Model output tensor data.
        /// @param num_boxes Number of candidate boxes in @p data.
        /// @return The detections surviving the confidence threshold and suppression pass.
        std::vector<detection_t> _postprocess(const float* data, std::size_t num_boxes) const;

        /// @brief Runs preprocessing, inference, and publishing over one image.
        /// @param bgr_image Image to run detection on.
        /// @param header Header of the message the image came from.
        void _run_inference_pipeline(const cv::Mat& bgr_image, const std_msgs::msg::Header& header);

        /// @brief Publishes image-space detections for the overlay node to draw.
        /// @param detections Detections found in the frame just processed.
        /// @param header Header of the message the image came from.
        /// @param detections Detections found in the frame just processed.
        /// @param header Header of the message the image came from.
        void _publish_detections(
            const std::vector<detection_t>& detections,
            const std_msgs::msg::Header& header);

        /// @brief Builds the human-readable summary returned by the detection service.
        /// @param detected_count Number of cubes found in the image.
        /// @param resolved_count Number of those cubes that got a depth-derived pose.
        /// @return A sentence describing how many detections were resolved.
        std::string _build_depth_summary_message(std::size_t detected_count, std::size_t resolved_count);

        /// @brief Warns when the depth and colour images are not in the same frame.
        ///
        /// Poses are derived from the depth intrinsics but published in the colour
        /// image's frame, so the streams must be aligned for the pose to be correct.
        /// Without this check a misconfigured depth topic is silently wrong by the
        /// colour-to-depth baseline rather than producing any visible symptom.
        /// @param color_header Header of the colour image being processed.
        void _warn_if_depth_frame_mismatched(const std_msgs::msg::Header& color_header);

        /// @brief Publishes rviz markers for every pose-resolved detection.
        /// @param detections Detections from the frame just processed.
        void _publish_markers(const perseus_interfaces::msg::DetectionArray& detections);

        /// @brief Replaces the cached detections served by the detection service.
        /// @param detections Detections that were just published.
        /// @param message Human-readable summary of the detection pass.
        void _cache_latest_detections(
            const perseus_interfaces::msg::DetectionArray& detections,
            std::string message);

        /// @brief Estimates a cube pose from the centre pixel of its bounding box.
        /// @param detection Detection to estimate a pose for.
        /// @param pose_out Receives the pose in the depth camera frame.
        /// @param distance_m_out Receives the straight-line distance to the cube, in metres.
        /// @return True if a valid, in-range depth reading was found.
        bool _estimate_cube_pose_from_depth(
            const detection_t& detection,
            geometry_msgs::msg::Pose& pose_out,
            double& distance_m_out) const;

        /// @brief Reads a median-filtered depth value at a pixel.
        /// @param depth_msg Depth image to sample.
        /// @param u Horizontal pixel coordinate.
        /// @param v Vertical pixel coordinate.
        /// @param depth_m_out Receives the depth in metres.
        /// @return True if the encoding is supported and at least one finite sample was read.
        bool _read_depth_meters_at_pixel(
            const sensor_msgs::msg::Image& depth_msg,
            int u,
            int v,
            double& depth_m_out) const;

        /// @brief Reads one depth pixel and converts it to metres.
        /// @param depth_msg Depth image to sample. Its encoding must already be known supported.
        /// @param row Pixel row to read.
        /// @param column Pixel column to read.
        /// @return The depth in metres, or NaN if the pixel holds no reading.
        double _read_depth_sample(const sensor_msgs::msg::Image& depth_msg, int row, int column) const;

        /// @brief Answers a detection service request from the cached detections.
        /// @param request Incoming request, which may also ask for an image capture.
        /// @param response Response populated with the cached detections.
        void _handle_detect_objects_request(
            const std::shared_ptr<DetectObjects::Request> request,
            std::shared_ptr<DetectObjects::Response> response);

        /// @brief Runs inference on the most recently received frame, if there is one.
        ///
        /// Used to serve requests on demand while the node is not always on.
        void _run_inference_on_cached_frame();

        /// @brief Writes an annotated capture of the latest detections to disk.
        /// @param save_path Directory to write the image into; `.` is used if empty.
        /// @param frame Raw frame to annotate and save. Modified in place.
        /// @param detections Detections to draw, rendered exactly as the overlay does.
        void _save_detection_capture(
            const std::string& save_path,
            cv::Mat& frame,
            const perseus_interfaces::msg::DetectionArray& detections) const;

        /// @brief Applies runtime updates to the reconfigurable parameters.
        /// @param parameters Parameters being set.
        /// @return Whether the update was accepted, and why if it was not.
        rcl_interfaces::msg::SetParametersResult _parameter_callback(
            const std::vector<rclcpp::Parameter>& parameters);

        // ONNX Runtime
        Ort::Env _ort_env;
        Ort::AllocatorWithDefaultOptions _ort_allocator;
        std::unique_ptr<Ort::Session> _ort_session;
        std::string _model_input_name;
        std::string _model_output_name;

        // Image size tracking for coordinate scaling
        int _original_height{0};
        int _original_width{0};
        int _padding_x{0};
        int _padding_y{0};
        float _letterbox_scale{1.0f};

        // Parameters
        std::string _model_path;
        std::string _output_detections_topic{DEFAULT_OUTPUT_DETECTIONS_TOPIC};
        std::string _depth_estimation_mode{DEFAULT_DEPTH_ESTIMATION_MODE};
        std::string _depth_topic{DEFAULT_DEPTH_TOPIC};
        std::string _depth_info_topic{DEFAULT_DEPTH_INFO_TOPIC};
        std::string _output_markers_topic{DEFAULT_OUTPUT_MARKERS_TOPIC};

        float _confidence_threshold{DEFAULT_CONFIDENCE_THRESHOLD};
        float _nms_iou_threshold{DEFAULT_NMS_IOU_THRESHOLD};
        std::atomic_bool _is_always_on{DEFAULT_IS_ALWAYS_ON};
        bool _should_use_cuda{DEFAULT_SHOULD_USE_CUDA};
        double _depth_unit_scale{DEFAULT_DEPTH_UNIT_SCALE};
        double _depth_max_range_m{DEFAULT_DEPTH_MAX_RANGE_M};
        double _depth_min_range_m{DEFAULT_DEPTH_MIN_RANGE_M};
        std::atomic<double> _processing_frequency_hz{DEFAULT_PROCESSING_FREQUENCY_HZ};
        int64_t _last_inference_time_ns{0};
        int _intra_op_num_threads{DEFAULT_INTRA_OP_NUM_THREADS};
        int _inter_op_num_threads{DEFAULT_INTER_OP_NUM_THREADS};
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr _param_callback_handle;

        // Camera calibration (kept for future use)
        std::mutex _camera_matrix_mutex;
        cv::Mat _camera_matrix;
        cv::Mat _distortion_coefficients;

        // Depth data
        mutable std::mutex _depth_mutex;
        sensor_msgs::msg::Image::SharedPtr _latest_depth_image;
        bool _has_depth_intrinsics{false};
        double _depth_fx{0.0};
        double _depth_fy{0.0};
        double _depth_cx{0.0};
        double _depth_cy{0.0};

        // Subscribers
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _image_subscription;
        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr _camera_info_subscription;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _depth_image_subscription;
        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr _depth_camera_info_subscription;

        // Publishers and services
        rclcpp::Publisher<perseus_interfaces::msg::DetectionArray>::SharedPtr _detection_publisher;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _cube_marker_publisher;
        rclcpp::Service<DetectObjects>::SharedPtr _detect_objects_service;

        // Cached frames and detections
        mutable std::mutex _latest_image_mutex;
        cv::Mat _latest_bgr_frame;
        std_msgs::msg::Header _latest_bgr_header;
        bool _has_latest_bgr_frame{false};
        mutable std::mutex _inference_mutex;
        mutable std::mutex _detections_mutex;
        perseus_interfaces::msg::DetectionArray _latest_detections;
        std::string _latest_detection_message{"No cube detections are currently cached."};
    };

}  // namespace perseus_vision
