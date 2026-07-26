/// @file cube_detector.cpp
/// @brief Implementation of the cube detection and depth-based pose estimation node.

#include "perseus_vision/cube_detector/cube_detector.hpp"

#include <tf2/exceptions.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>

namespace perseus_vision
{
    namespace
    {
        /// @brief Queue depth used for all image, detection, and marker topics.
        constexpr int QOS_DEPTH = 10;
        /// @brief Square side length, in pixels, that images are letterboxed to for inference.
        constexpr int MODEL_INPUT_SIZE_PX = 640;
        /// @brief Grey value YOLOv8 expects in the letterbox padding region.
        constexpr double LETTERBOX_PADDING_VALUE = 114.0;
        /// @brief Number of colour channels in a model input tensor.
        constexpr int64_t MODEL_INPUT_CHANNELS = 3;
        /// @brief Leading box values (center x, center y, width, height) in each model output column.
        constexpr std::size_t BOX_COORDINATE_COUNT = 4;
        /// @brief Largest value a single 8-bit colour channel can hold.
        constexpr double PIXEL_VALUE_MAX = 255.0;
        /// @brief Nanoseconds in one second, used to convert a rate limit into a period.
        constexpr double NANOSECONDS_PER_SECOND = 1e9;
        /// @brief Minimum gap between repeated warning logs, in milliseconds.
        constexpr int64_t LOG_THROTTLE_MS = 2000;
        /// @brief Half-width of the square pixel window sampled when reading a depth value.
        constexpr int DEPTH_SAMPLE_RADIUS_PX = 1;

        /// @brief Edge length of the cube drawn for each detection in rviz, in metres.
        constexpr double CUBE_MARKER_SIZE_M = 0.12;
        /// @brief How high above the cube its text label floats, in metres.
        constexpr double TEXT_MARKER_HEIGHT_OFFSET_M = 0.12;
        /// @brief Cap height of the cube text labels, in metres.
        constexpr double TEXT_MARKER_SCALE_M = 0.08;
        /// @brief How long rviz keeps a cube marker before treating it as stale, in seconds.
        constexpr double MARKER_LIFETIME_S = 0.2;
        /// @brief Opacity of the cube markers drawn in rviz.
        constexpr float MARKER_ALPHA = 0.85f;
        /// @brief rviz namespace holding the cube markers.
        const std::string CUBE_MARKER_NAMESPACE = "cube_depth";
        /// @brief rviz namespace holding the cube text labels.
        const std::string CUBE_TEXT_MARKER_NAMESPACE = "cube_depth_text";

        /// @brief Value of the `depth_estimation_mode` parameter that enables depth image poses.
        const std::string DEPTH_MODE_DEPTH_IMAGE = "depth_image";
        /// @brief File name of the cube detection model shipped in the package share directory.
        const std::string PACKAGED_MODEL_FILENAME = "cube_detector_yolob8s.onnx";
        /// @brief Name of the package the model and configuration are installed under.
        const std::string PACKAGE_NAME = "perseus_vision";

        /// @brief Depth image encoding holding 16-bit unsigned values.
        const std::string DEPTH_ENCODING_16UC1 = "16UC1";
        /// @brief Depth image encoding holding 32-bit float metres.
        const std::string DEPTH_ENCODING_32FC1 = "32FC1";

        /// @brief Line thickness of a detection bounding box, in pixels.
        constexpr int BOX_THICKNESS = 2;
        /// @brief Height of the class label above the bounding box, in pixels.
        constexpr int LABEL_Y_OFFSET_PX = 20;
        /// @brief Height of the center coordinate label above the bounding box, in pixels.
        constexpr int CENTER_LABEL_Y_OFFSET_PX = 5;
        /// @brief Font scale of the class label.
        constexpr double LABEL_FONT_SCALE = 0.6;
        /// @brief Font scale of the center coordinate label.
        constexpr double CENTER_LABEL_FONT_SCALE = 0.5;
        /// @brief Stroke thickness of the class label.
        constexpr int LABEL_THICKNESS = 2;
        /// @brief Stroke thickness of the center coordinate label.
        constexpr int CENTER_LABEL_THICKNESS = 1;
        /// @brief Arm length of the crosshair drawn at a detection center, in pixels.
        constexpr int CROSSHAIR_SIZE_PX = 15;
        /// @brief Stroke thickness of the detection center crosshair.
        constexpr int CROSSHAIR_THICKNESS = 1;
        /// @brief Characters of the confidence value kept in a label, as in "0.97".
        constexpr std::size_t CONFIDENCE_TEXT_LENGTH = 4;

        /// @brief Left margin used for all capture overlay text, in pixels.
        constexpr int OVERLAY_MARGIN_PX = 10;
        /// @brief Baseline of the capture timestamp line, in pixels from the top.
        constexpr int OVERLAY_TIMESTAMP_Y_PX = 25;
        /// @brief Baseline of the frame ID line, in pixels from the top.
        constexpr int OVERLAY_FRAME_Y_PX = 50;
        /// @brief Vertical spacing between detection lines, in pixels.
        constexpr int OVERLAY_LINE_HEIGHT_PX = 20;
        /// @brief Font scale used for the capture timestamp.
        constexpr double OVERLAY_TIMESTAMP_FONT_SCALE = 0.6;
        /// @brief Font scale used for the frame ID and detection lines.
        constexpr double OVERLAY_TEXT_FONT_SCALE = 0.5;
        /// @brief Stroke thickness used for the capture timestamp.
        constexpr int OVERLAY_TIMESTAMP_THICKNESS = 2;
        /// @brief Stroke thickness used for the frame ID and detection lines.
        constexpr int OVERLAY_TEXT_THICKNESS = 1;
        /// @brief Decimal places shown for pose coordinates in text.
        constexpr int POSE_TEXT_PRECISION = 2;
        /// @brief Longest formatted timestamp, including the terminator.
        constexpr std::size_t TIMESTAMP_BUFFER_SIZE = 64;

        /// @brief The best scoring class for a single candidate box.
        struct class_score_t
        {
            /// @brief Index into CLASS_NAMES and CLASS_COLORS.
            std::size_t class_id{0};
            /// @brief Score the model gave that class.
            float score{0.0f};
        };

        /// @brief Converts an image-space bounding box into a region of interest message.
        /// @param bounding_box Bounding box in original image coordinates.
        /// @return The equivalent region of interest, clamped to non-negative offsets.
        sensor_msgs::msg::RegionOfInterest region_of_interest_from_rect(const cv::Rect& bounding_box)
        {
            sensor_msgs::msg::RegionOfInterest region_of_interest;
            region_of_interest.x_offset = static_cast<uint32_t>(std::max(0, bounding_box.x));
            region_of_interest.y_offset = static_cast<uint32_t>(std::max(0, bounding_box.y));
            region_of_interest.width = static_cast<uint32_t>(std::max(0, bounding_box.width));
            region_of_interest.height = static_cast<uint32_t>(std::max(0, bounding_box.height));
            region_of_interest.do_rectify = false;
            return region_of_interest;
        }

        /// @brief Finds the highest scoring class for one candidate box in the model output.
        /// @param data Model output tensor data.
        /// @param num_boxes Number of candidate boxes in @p data.
        /// @param box_index Index of the candidate box to score.
        /// @return The best scoring class and its score.
        class_score_t best_class_for_box(const float* data, std::size_t num_boxes, std::size_t box_index)
        {
            class_score_t best;
            for (std::size_t class_index = 0; class_index < NUM_CLASSES; ++class_index)
            {
                const float score = data[((BOX_COORDINATE_COUNT + class_index) * num_boxes) + box_index];
                if (score > best.score)
                {
                    best.score = score;
                    best.class_id = class_index;
                }
            }
            return best;
        }

        /// @brief Builds a pose whose every field is NaN, marking it as unresolved.
        /// @return A pose signalling that no 3D position is available.
        geometry_msgs::msg::Pose make_unresolved_pose()
        {
            const double not_a_number = std::numeric_limits<double>::quiet_NaN();

            geometry_msgs::msg::Pose pose;
            pose.position.x = not_a_number;
            pose.position.y = not_a_number;
            pose.position.z = not_a_number;
            pose.orientation.x = not_a_number;
            pose.orientation.y = not_a_number;
            pose.orientation.z = not_a_number;
            pose.orientation.w = not_a_number;
            return pose;
        }

        /// @brief Builds the quaternion pointing along a given yaw and pitch.
        /// @param yaw Rotation about the vertical axis, in radians.
        /// @param pitch Rotation about the lateral axis, in radians.
        /// @return The equivalent orientation.
        geometry_msgs::msg::Quaternion quaternion_from_yaw_pitch(double yaw, double pitch)
        {
            const double half_yaw = 0.5 * yaw;
            const double half_pitch = 0.5 * pitch;
            const double cos_half_yaw = std::cos(half_yaw);
            const double sin_half_yaw = std::sin(half_yaw);
            const double cos_half_pitch = std::cos(half_pitch);
            const double sin_half_pitch = std::sin(half_pitch);

            geometry_msgs::msg::Quaternion quaternion;
            quaternion.w = cos_half_yaw * cos_half_pitch;
            quaternion.x = -sin_half_yaw * sin_half_pitch;
            quaternion.y = cos_half_yaw * sin_half_pitch;
            quaternion.z = sin_half_yaw * cos_half_pitch;
            return quaternion;
        }

        /// @brief Builds the marker that clears the previous frame's cube markers from rviz.
        /// @param header Header stamped onto the marker.
        /// @return A DELETEALL marker for the cube namespace.
        visualization_msgs::msg::Marker make_clear_marker(const std_msgs::msg::Header& header)
        {
            visualization_msgs::msg::Marker marker;
            marker.header = header;
            marker.ns = CUBE_MARKER_NAMESPACE;
            marker.id = 0;
            marker.action = visualization_msgs::msg::Marker::DELETEALL;
            return marker;
        }

        /// @brief Builds the rviz cube drawn at a detected cube's estimated pose.
        /// @param header Header stamped onto the marker.
        /// @param marker_id Unique ID within the cube marker namespace.
        /// @param pose Estimated cube pose in the marker's frame.
        /// @param class_id Cube class, used to colour the marker.
        /// @return The cube marker.
        visualization_msgs::msg::Marker make_cube_marker(
            const std_msgs::msg::Header& header,
            int32_t marker_id,
            const geometry_msgs::msg::Pose& pose,
            int32_t class_id)
        {
            const cv::Scalar& color = CLASS_COLORS[static_cast<std::size_t>(class_id)];

            visualization_msgs::msg::Marker marker;
            marker.header = header;
            marker.ns = CUBE_MARKER_NAMESPACE;
            marker.id = marker_id;
            marker.type = visualization_msgs::msg::Marker::CUBE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose = pose;
            marker.scale.x = CUBE_MARKER_SIZE_M;
            marker.scale.y = CUBE_MARKER_SIZE_M;
            marker.scale.z = CUBE_MARKER_SIZE_M;
            marker.color.a = MARKER_ALPHA;
            // CLASS_COLORS holds BGR values, while marker colours are RGB.
            marker.color.r = static_cast<float>(color[2] / PIXEL_VALUE_MAX);
            marker.color.g = static_cast<float>(color[1] / PIXEL_VALUE_MAX);
            marker.color.b = static_cast<float>(color[0] / PIXEL_VALUE_MAX);
            marker.lifetime = rclcpp::Duration::from_seconds(MARKER_LIFETIME_S);
            return marker;
        }

        /// @brief Builds the rviz text label floating above a detected cube.
        /// @param header Header stamped onto the marker.
        /// @param marker_id Unique ID within the cube text marker namespace.
        /// @param pose Estimated cube pose in the marker's frame.
        /// @param class_id Cube class, used to name the cube in the label.
        /// @return The text marker.
        visualization_msgs::msg::Marker make_text_marker(
            const std_msgs::msg::Header& header,
            int32_t marker_id,
            const geometry_msgs::msg::Pose& pose,
            int32_t class_id)
        {
            visualization_msgs::msg::Marker marker;
            marker.header = header;
            marker.ns = CUBE_TEXT_MARKER_NAMESPACE;
            marker.id = marker_id;
            marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose = pose;
            marker.pose.position.z += TEXT_MARKER_HEIGHT_OFFSET_M;
            marker.scale.z = TEXT_MARKER_SCALE_M;
            marker.color.a = 1.0f;
            marker.color.r = 1.0f;
            marker.color.g = 1.0f;
            marker.color.b = 1.0f;

            std::ostringstream text_stream;
            text_stream << CLASS_NAMES[static_cast<std::size_t>(class_id)]
                        << " x:" << std::fixed << std::setprecision(POSE_TEXT_PRECISION) << pose.position.x
                        << " y:" << std::fixed << std::setprecision(POSE_TEXT_PRECISION) << pose.position.y;
            marker.text = text_stream.str();

            marker.lifetime = rclcpp::Duration::from_seconds(MARKER_LIFETIME_S);
            return marker;
        }

        /// @brief Formats the current local time for display on a captured image.
        /// @return The time formatted as `YYYY-MM-DD HH:MM:SS`.
        std::string format_local_timestamp()
        {
            const auto now = std::chrono::system_clock::now();
            const std::time_t now_seconds = std::chrono::system_clock::to_time_t(now);

            // localtime_r is used over localtime as the latter is not thread safe.
            std::tm local_time{};
            localtime_r(&now_seconds, &local_time);

            char timestamp_text[TIMESTAMP_BUFFER_SIZE];
            std::strftime(timestamp_text, sizeof(timestamp_text), "%Y-%m-%d %H:%M:%S", &local_time);
            return std::string(timestamp_text);
        }

        /// @brief Draws the capture timestamp, frame ID, and detection list onto an image.
        /// @param frame Image to draw onto. Modified in place.
        /// @param ids Detection IDs to list.
        /// @param poses Detection poses matching @p ids.
        /// @param bounding_boxes Bounding boxes matching @p ids, drawn onto the image.
        /// @param frame_id Frame the poses are expressed in.
        void draw_detection_overlay(
            cv::Mat& frame,
            const std::vector<int32_t>& ids,
            const std::vector<geometry_msgs::msg::Pose>& poses,
            const std::vector<cv::Rect>& bounding_boxes,
            const std::string& frame_id)
        {
            const cv::Scalar timestamp_color(0, 255, 255);
            const cv::Scalar frame_id_color(255, 255, 0);
            const cv::Scalar detection_color(0, 255, 0);
            const cv::Scalar bounding_box_color(0, 255, 255);

            cv::putText(frame, "Time: " + format_local_timestamp(),
                        cv::Point(OVERLAY_MARGIN_PX, OVERLAY_TIMESTAMP_Y_PX),
                        cv::FONT_HERSHEY_SIMPLEX, OVERLAY_TIMESTAMP_FONT_SCALE,
                        timestamp_color, OVERLAY_TIMESTAMP_THICKNESS);

            int text_y = OVERLAY_FRAME_Y_PX;
            cv::putText(frame, "Frame: " + frame_id,
                        cv::Point(OVERLAY_MARGIN_PX, text_y),
                        cv::FONT_HERSHEY_SIMPLEX, OVERLAY_TEXT_FONT_SCALE,
                        frame_id_color, OVERLAY_TEXT_THICKNESS);

            for (std::size_t i = 0; i < ids.size() && i < poses.size(); ++i)
            {
                if (i < bounding_boxes.size())
                {
                    cv::rectangle(frame, bounding_boxes[i], bounding_box_color, BOX_THICKNESS);
                }

                text_y += OVERLAY_LINE_HEIGHT_PX;
                std::ostringstream detection_line;
                detection_line << "ID " << ids[i]
                               << " x=" << std::fixed << std::setprecision(POSE_TEXT_PRECISION) << poses[i].position.x
                               << " y=" << std::fixed << std::setprecision(POSE_TEXT_PRECISION) << poses[i].position.y;
                cv::putText(frame, detection_line.str(),
                            cv::Point(OVERLAY_MARGIN_PX, text_y),
                            cv::FONT_HERSHEY_SIMPLEX, OVERLAY_TEXT_FONT_SCALE,
                            detection_color, OVERLAY_TEXT_THICKNESS);
            }
        }
    }  // namespace

    CubeDetector::CubeDetector()
        : Node("cube_detector"),
          _ort_env(ORT_LOGGING_LEVEL_WARNING, "cube_detector")
    {
        std::string camera_topic;
        std::string camera_info_topic;
        _load_parameters(camera_topic, camera_info_topic);

        _model_path = _resolve_model_path();
        _load_model();

        // Subscribers
        _image_subscription = create_subscription<sensor_msgs::msg::Image>(
            camera_topic, QOS_DEPTH,
            std::bind(&CubeDetector::_image_callback, this, std::placeholders::_1));

        _camera_info_subscription = create_subscription<sensor_msgs::msg::CameraInfo>(
            camera_info_topic, QOS_DEPTH,
            std::bind(&CubeDetector::_camera_info_callback, this, std::placeholders::_1));

        if (_depth_estimation_mode == DEPTH_MODE_DEPTH_IMAGE)
        {
            _depth_image_subscription = create_subscription<sensor_msgs::msg::Image>(
                _depth_topic, QOS_DEPTH,
                std::bind(&CubeDetector::_depth_image_callback, this, std::placeholders::_1));
            _depth_camera_info_subscription = create_subscription<sensor_msgs::msg::CameraInfo>(
                _depth_info_topic, QOS_DEPTH,
                std::bind(&CubeDetector::_depth_camera_info_callback, this, std::placeholders::_1));
        }

        // Publishers and services
        _annotated_image_publisher = create_publisher<sensor_msgs::msg::Image>(
            _output_img_topic, QOS_DEPTH);
        _cube_marker_publisher = create_publisher<visualization_msgs::msg::MarkerArray>(
            _output_markers_topic, QOS_DEPTH);
        _cube_detection_publisher = create_publisher<perseus_interfaces::msg::ObjectDetections>(
            _output_detections_topic, QOS_DEPTH);
        _detect_objects_service = create_service<DetectObjects>(
            "~/detect_objects",
            std::bind(&CubeDetector::_handle_detect_objects_request, this,
                      std::placeholders::_1, std::placeholders::_2));

        _tf_buffer = std::make_unique<tf2_ros::Buffer>(get_clock());
        _tf_listener = std::make_shared<tf2_ros::TransformListener>(*_tf_buffer);

        RCLCPP_INFO(get_logger(), "CubeDetectorNode ready — subscribed to %s", camera_topic.c_str());
        if (_is_always_on.load())
        {
            RCLCPP_INFO(get_logger(), "Node is set to always_on, will process images even without subscribers");
        }
        else
        {
            RCLCPP_INFO(get_logger(), "Always on is disabled!");
        }
    }

    void CubeDetector::_load_parameters(std::string& camera_topic_out, std::string& camera_info_topic_out)
    {
        declare_parameter("model_path", "");
        declare_parameter("confidence_threshold", static_cast<double>(DEFAULT_CONFIDENCE_THRESHOLD));
        declare_parameter("camera_topic", DEFAULT_CAMERA_TOPIC);
        declare_parameter("camera_info_topic", DEFAULT_CAMERA_INFO_TOPIC);
        // Keep the node alive even without subscribers
        declare_parameter("always_on", DEFAULT_IS_ALWAYS_ON);
        declare_parameter("use_cuda", DEFAULT_SHOULD_USE_CUDA);
        declare_parameter("publish_annotated_image", DEFAULT_SHOULD_PUBLISH_ANNOTATED_IMAGE);
        // Zero processes every frame
        declare_parameter("processing_frequency_hz", DEFAULT_PROCESSING_FREQUENCY_HZ);
        declare_parameter("intra_op_num_threads", DEFAULT_INTRA_OP_NUM_THREADS);
        declare_parameter("inter_op_num_threads", DEFAULT_INTER_OP_NUM_THREADS);
        declare_parameter("nms_iou_threshold", static_cast<double>(DEFAULT_NMS_IOU_THRESHOLD));
        declare_parameter("depth_estimation_mode", DEFAULT_DEPTH_ESTIMATION_MODE);
        declare_parameter("depth_image.topic", DEFAULT_DEPTH_TOPIC);
        declare_parameter("depth_image.info_topic", DEFAULT_DEPTH_INFO_TOPIC);
        declare_parameter("depth_image.unit_scale", DEFAULT_DEPTH_UNIT_SCALE);
        declare_parameter("depth_image.max_range_m", DEFAULT_DEPTH_MAX_RANGE_M);
        declare_parameter("depth_image.min_range_m", DEFAULT_DEPTH_MIN_RANGE_M);
        declare_parameter("tf_output_frame", DEFAULT_TF_OUTPUT_FRAME);
        declare_parameter("output_img_topic", DEFAULT_OUTPUT_IMG_TOPIC);
        declare_parameter("output_detections_topic", DEFAULT_OUTPUT_DETECTIONS_TOPIC);
        declare_parameter("output_markers_topic", DEFAULT_OUTPUT_MARKERS_TOPIC);

        camera_topic_out = get_parameter("camera_topic").as_string();
        camera_info_topic_out = get_parameter("camera_info_topic").as_string();
        _intra_op_num_threads = static_cast<int>(get_parameter("intra_op_num_threads").as_int());
        _inter_op_num_threads = static_cast<int>(get_parameter("inter_op_num_threads").as_int());
        _nms_iou_threshold = static_cast<float>(get_parameter("nms_iou_threshold").as_double());
        _depth_estimation_mode = get_parameter("depth_estimation_mode").as_string();
        _depth_topic = get_parameter("depth_image.topic").as_string();
        _depth_info_topic = get_parameter("depth_image.info_topic").as_string();
        _depth_unit_scale = get_parameter("depth_image.unit_scale").as_double();
        _depth_max_range_m = get_parameter("depth_image.max_range_m").as_double();
        _depth_min_range_m = get_parameter("depth_image.min_range_m").as_double();
        _confidence_threshold = static_cast<float>(get_parameter("confidence_threshold").as_double());
        _is_always_on.store(get_parameter("always_on").as_bool());
        _should_use_cuda = get_parameter("use_cuda").as_bool();
        _should_publish_annotated_image = get_parameter("publish_annotated_image").as_bool();
        _processing_frequency_hz.store(get_parameter("processing_frequency_hz").as_double());
        _model_path = get_parameter("model_path").as_string();
        _tf_output_frame = get_parameter("tf_output_frame").as_string();
        _output_img_topic = get_parameter("output_img_topic").as_string();
        _output_detections_topic = get_parameter("output_detections_topic").as_string();
        _output_markers_topic = get_parameter("output_markers_topic").as_string();

        _param_callback_handle = add_on_set_parameters_callback(
            std::bind(&CubeDetector::_parameter_callback, this, std::placeholders::_1));
    }

    std::string CubeDetector::_resolve_model_path() const
    {
        std::string model_path = _model_path;
        const std::string package_models_directory =
            ament_index_cpp::get_package_share_directory(PACKAGE_NAME) + "/models";

        if (model_path.rfind("~/", 0) == 0)
        {
            if (const char* home = std::getenv("HOME"))
            {
                model_path = std::string(home) + model_path.substr(1);
            }
        }

        // Backward compatibility: some configs use "/perseus-v2/..." (missing $HOME prefix).
        if (!model_path.empty() && !std::filesystem::exists(model_path))
        {
            if (const char* home = std::getenv("HOME"))
            {
                const std::string home_prefixed = std::string(home) + model_path;
                if (std::filesystem::exists(home_prefixed))
                {
                    model_path = home_prefixed;
                }
            }
        }

        // Resolve bare relative filenames from the installed package models directory.
        if (!model_path.empty() && !std::filesystem::path(model_path).is_absolute() &&
            !std::filesystem::exists(model_path))
        {
            const std::string packaged_model_candidate = package_models_directory + "/" + model_path;
            if (std::filesystem::exists(packaged_model_candidate))
            {
                model_path = packaged_model_candidate;
            }
        }

        if (model_path.empty() || !std::filesystem::exists(model_path))
        {
            const std::string packaged_model = package_models_directory + "/" + PACKAGED_MODEL_FILENAME;
            if (std::filesystem::exists(packaged_model))
            {
                model_path = packaged_model;
            }
        }

        return model_path;
    }

    void CubeDetector::_load_model()
    {
        RCLCPP_INFO(get_logger(), "Loading model: %s", _model_path.c_str());

        Ort::SessionOptions session_options;
        session_options.SetIntraOpNumThreads(_intra_op_num_threads);
        session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);
        session_options.SetInterOpNumThreads(_inter_op_num_threads);

        if (_should_use_cuda)
        {
            OrtCUDAProviderOptions cuda_options{};
            try
            {
                session_options.AppendExecutionProvider_CUDA(cuda_options);
                RCLCPP_INFO(get_logger(), "Using CUDA execution provider");
            }
            catch (const Ort::Exception&)
            {
                RCLCPP_WARN(get_logger(), "CUDA unavailable, falling back to CPU");
            }
        }

        try
        {
            _ort_session = std::make_unique<Ort::Session>(
                _ort_env, _model_path.c_str(), session_options);
        }
        catch (const Ort::Exception& e)
        {
            RCLCPP_FATAL(get_logger(), "Failed to load ONNX model from '%s': %s", _model_path.c_str(), e.what());
            throw;
        }

        const auto input_name = _ort_session->GetInputNameAllocated(0, _ort_allocator);
        const auto output_name = _ort_session->GetOutputNameAllocated(0, _ort_allocator);
        _model_input_name = input_name.get();
        _model_output_name = output_name.get();

        RCLCPP_INFO(get_logger(), "Model loaded — input: %s  output: %s",
                    _model_input_name.c_str(), _model_output_name.c_str());
    }

    void CubeDetector::_image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv_bridge::CvImagePtr cv_image;
        try
        {
            cv_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (const cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        // Always cache latest frame so service can run on-demand even when always_on is false.
        {
            std::lock_guard<std::mutex> lock(_latest_image_mutex);
            _latest_bgr_frame = cv_image->image.clone();
            _latest_bgr_header = msg->header;
            _has_latest_bgr_frame = true;
        }

        if (!_is_always_on.load() || !_should_run_inference_now())
        {
            return;
        }

        std::lock_guard<std::mutex> lock(_inference_mutex);
        _run_inference_pipeline(cv_image->image, msg->header);
    }

    bool CubeDetector::_should_run_inference_now()
    {
        const double processing_frequency_hz = _processing_frequency_hz.load();
        if (processing_frequency_hz <= 0.0)
        {
            return true;
        }

        const int64_t now_ns = this->now().nanoseconds();
        // Reset the rate limiter if the clock jumped backwards, such as on a sim time reset.
        if (_last_inference_time_ns != 0 && now_ns < _last_inference_time_ns)
        {
            _last_inference_time_ns = 0;
        }

        const int64_t min_period_ns = static_cast<int64_t>(NANOSECONDS_PER_SECOND / processing_frequency_hz);
        if (_last_inference_time_ns != 0 && (now_ns - _last_inference_time_ns) < min_period_ns)
        {
            return false;
        }

        _last_inference_time_ns = now_ns;
        return true;
    }

    void CubeDetector::_camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(_camera_matrix_mutex);

        constexpr int camera_matrix_side = 3;
        _camera_matrix = cv::Mat(camera_matrix_side, camera_matrix_side, CV_64F);
        for (int row = 0; row < camera_matrix_side; ++row)
        {
            for (int column = 0; column < camera_matrix_side; ++column)
            {
                _camera_matrix.at<double>(row, column) = msg->k[(row * camera_matrix_side) + column];
            }
        }

        _distortion_coefficients = cv::Mat(static_cast<int>(msg->d.size()), 1, CV_64F);
        for (std::size_t i = 0; i < msg->d.size(); ++i)
        {
            _distortion_coefficients.at<double>(static_cast<int>(i), 0) = msg->d[i];
        }
    }

    void CubeDetector::_depth_image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(_depth_mutex);
        _latest_depth_image = msg;
    }

    void CubeDetector::_depth_camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(_depth_mutex);
        _depth_fx = msg->k[0];
        _depth_fy = msg->k[4];
        _depth_cx = msg->k[2];
        _depth_cy = msg->k[5];
        _has_depth_intrinsics = _depth_fx > 0.0 && _depth_fy > 0.0;
    }

    std::vector<float> CubeDetector::_preprocess(const cv::Mat& bgr_image)
    {
        _original_height = bgr_image.rows;
        _original_width = bgr_image.cols;

        // Letterbox, preserving the aspect ratio
        const float scale = std::min(static_cast<float>(MODEL_INPUT_SIZE_PX) / static_cast<float>(_original_width),
                                     static_cast<float>(MODEL_INPUT_SIZE_PX) / static_cast<float>(_original_height));
        const int scaled_width = static_cast<int>(static_cast<float>(_original_width) * scale);
        const int scaled_height = static_cast<int>(static_cast<float>(_original_height) * scale);

        _padding_x = (MODEL_INPUT_SIZE_PX - scaled_width) / 2;
        _padding_y = (MODEL_INPUT_SIZE_PX - scaled_height) / 2;
        _letterbox_scale = scale;

        cv::Mat resized;
        cv::resize(bgr_image, resized, cv::Size(scaled_width, scaled_height));

        cv::Mat padded(MODEL_INPUT_SIZE_PX, MODEL_INPUT_SIZE_PX, CV_8UC3, cv::Scalar::all(LETTERBOX_PADDING_VALUE));
        resized.copyTo(padded(cv::Rect(_padding_x, _padding_y, scaled_width, scaled_height)));

        // BGR -> RGB -> float -> CHW
        cv::Mat rgb;
        cv::cvtColor(padded, rgb, cv::COLOR_BGR2RGB);
        rgb.convertTo(rgb, CV_32FC3, 1.0 / PIXEL_VALUE_MAX);

        std::vector<cv::Mat> channels(static_cast<std::size_t>(MODEL_INPUT_CHANNELS));
        cv::split(rgb, channels);

        constexpr int pixels_per_channel = MODEL_INPUT_SIZE_PX * MODEL_INPUT_SIZE_PX;
        std::vector<float> blob;
        blob.reserve(static_cast<std::size_t>(MODEL_INPUT_CHANNELS) * pixels_per_channel);
        for (const auto& channel : channels)
        {
            const float* channel_data = reinterpret_cast<const float*>(channel.data);
            blob.insert(blob.end(), channel_data, channel_data + pixels_per_channel);
        }
        return blob;
    }

    std::vector<detection_t> CubeDetector::_postprocess(const float* data, std::size_t num_boxes) const
    {
        std::vector<cv::Rect> boxes;
        std::vector<float> scores;
        std::vector<int32_t> class_ids;

        for (std::size_t i = 0; i < num_boxes; ++i)
        {
            const float center_x = data[(0 * num_boxes) + i];
            const float center_y = data[(1 * num_boxes) + i];
            const float width = data[(2 * num_boxes) + i];
            const float height = data[(3 * num_boxes) + i];

            const class_score_t best = best_class_for_box(data, num_boxes, i);
            if (best.score < _confidence_threshold)
            {
                continue;
            }

            // YOLOv8 outputs box coordinates in the letterboxed space, so the letterbox
            // transformation is reversed here to get back to original image coordinates.
            const int x1 = std::clamp(static_cast<int>((center_x - (width / 2.0f) - static_cast<float>(_padding_x)) / _letterbox_scale), 0, _original_width);
            const int y1 = std::clamp(static_cast<int>((center_y - (height / 2.0f) - static_cast<float>(_padding_y)) / _letterbox_scale), 0, _original_height);
            const int x2 = std::clamp(static_cast<int>((center_x + (width / 2.0f) - static_cast<float>(_padding_x)) / _letterbox_scale), 0, _original_width);
            const int y2 = std::clamp(static_cast<int>((center_y + (height / 2.0f) - static_cast<float>(_padding_y)) / _letterbox_scale), 0, _original_height);

            boxes.push_back(cv::Rect(x1, y1, x2 - x1, y2 - y1));
            scores.push_back(best.score);
            class_ids.push_back(static_cast<int32_t>(best.class_id));
        }

        // Suppress non-maximal boxes, guarding against an empty candidate list
        std::vector<int> indices;
        if (!boxes.empty())
        {
            cv::dnn::NMSBoxes(boxes, scores, _confidence_threshold, _nms_iou_threshold, indices);
        }

        std::vector<detection_t> detections;
        detections.reserve(indices.size());
        for (const int index : indices)
        {
            const std::size_t detection_index = static_cast<std::size_t>(index);
            detections.push_back({class_ids[detection_index], scores[detection_index], boxes[detection_index]});
        }

        return detections;
    }

    void CubeDetector::_run_inference_pipeline(
        const cv::Mat& bgr_image,
        const std_msgs::msg::Header& header)
    {
        std::vector<float> blob = _preprocess(bgr_image);

        const std::array<int64_t, 4> input_shape = {
            1, MODEL_INPUT_CHANNELS, MODEL_INPUT_SIZE_PX, MODEL_INPUT_SIZE_PX};
        const Ort::MemoryInfo memory_info =
            Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);

        Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
            memory_info, blob.data(), blob.size(),
            input_shape.data(), input_shape.size());

        const char* input_names[] = {_model_input_name.c_str()};
        const char* output_names[] = {_model_output_name.c_str()};

        const auto output_tensors = _ort_session->Run(
            Ort::RunOptions{nullptr},
            input_names, &input_tensor, 1,
            output_names, 1);

        const float* output_data = output_tensors[0].GetTensorData<float>();
        const auto output_shape = output_tensors[0].GetTensorTypeAndShapeInfo().GetShape();
        const std::size_t num_boxes = static_cast<std::size_t>(output_shape[2]);

        const std::vector<detection_t> detections = _postprocess(output_data, num_boxes);
        _publish_annotated_image(bgr_image, detections, header);

        if (_depth_estimation_mode == DEPTH_MODE_DEPTH_IMAGE)
        {
            _publish_depth_markers(detections, header);
            return;
        }

        _publish_2d_detections(detections, header);
    }

    void CubeDetector::_publish_annotated_image(
        const cv::Mat& image,
        const std::vector<detection_t>& detections,
        const std_msgs::msg::Header& header)
    {
        cv::Mat annotated = image.clone();

        for (const auto& detection : detections)
        {
            const std::size_t class_index = static_cast<std::size_t>(detection.class_id);
            const cv::Scalar& color = CLASS_COLORS[class_index];

            cv::rectangle(annotated, detection.bounding_box, color, BOX_THICKNESS);

            // Class and confidence, drawn above the box, such as "blue 0.97"
            const std::string label =
                CLASS_NAMES[class_index] + " " + std::to_string(detection.confidence).substr(0, CONFIDENCE_TEXT_LENGTH);
            cv::putText(annotated, label,
                        cv::Point(detection.bounding_box.x, detection.bounding_box.y - LABEL_Y_OFFSET_PX),
                        cv::FONT_HERSHEY_SIMPLEX, LABEL_FONT_SCALE, color, LABEL_THICKNESS);

            // Center coordinates, drawn below the class label, such as "cx:320 cy:240"
            const int center_x = detection.bounding_box.x + (detection.bounding_box.width / 2);
            const int center_y = detection.bounding_box.y + (detection.bounding_box.height / 2);
            const std::string center_label =
                "cx:" + std::to_string(center_x) + " cy:" + std::to_string(center_y);
            cv::putText(annotated, center_label,
                        cv::Point(detection.bounding_box.x, detection.bounding_box.y - CENTER_LABEL_Y_OFFSET_PX),
                        cv::FONT_HERSHEY_SIMPLEX, CENTER_LABEL_FONT_SCALE, color, CENTER_LABEL_THICKNESS);

            cv::drawMarker(annotated, cv::Point(center_x, center_y), color,
                           cv::MARKER_CROSS, CROSSHAIR_SIZE_PX, CROSSHAIR_THICKNESS);
        }

        if (_should_publish_annotated_image)
        {
            _annotated_image_publisher->publish(
                *cv_bridge::CvImage(header, "bgr8", annotated).toImageMsg());
        }

        {
            std::lock_guard<std::mutex> lock(_detections_mutex);
            _latest_annotated_frame = annotated.clone();
        }
    }

    void CubeDetector::_get_depth_source(
        const std_msgs::msg::Header& header,
        std::string& source_frame_out,
        builtin_interfaces::msg::Time& source_stamp_out) const
    {
        source_frame_out = header.frame_id;
        source_stamp_out = header.stamp;

        std::lock_guard<std::mutex> lock(_depth_mutex);
        if (_latest_depth_image)
        {
            source_frame_out = _latest_depth_image->header.frame_id;
            source_stamp_out = _latest_depth_image->header.stamp;
        }
    }

    std::string CubeDetector::_get_tf_output_frame() const
    {
        std::lock_guard<std::mutex> lock(_parameter_mutex);
        return _tf_output_frame;
    }

    std::string CubeDetector::_build_depth_summary_message(std::size_t detected_count, std::size_t resolved_count)
    {
        if (detected_count == 0)
        {
            return "No cube detections found in the latest image.";
        }

        std::ostringstream message_stream;
        if (resolved_count == 0)
        {
            message_stream << "Detected " << detected_count
                           << " cube(s) in 2D, but depth estimation failed for all detections. "
                           << "Returning 0 depth-resolved cube pose(s).";
            const std::string message = message_stream.str();
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), LOG_THROTTLE_MS, "%s", message.c_str());
            return message;
        }

        if (resolved_count < detected_count)
        {
            message_stream << "Resolved " << resolved_count << " of " << detected_count
                           << " detected cube(s) to depth-derived poses.";
            return message_stream.str();
        }

        message_stream << "Resolved all " << resolved_count << " detected cube(s) to depth-derived poses.";
        return message_stream.str();
    }

    void CubeDetector::_cache_latest_detections(
        const perseus_interfaces::msg::ObjectDetections& detections_msg,
        std::vector<cv::Rect> bounding_boxes,
        std::string message)
    {
        std::lock_guard<std::mutex> lock(_detections_mutex);
        _latest_detections_stamp = detections_msg.stamp;
        _latest_detections_frame_id = detections_msg.frame_id;
        _latest_detection_ids = detections_msg.ids;
        _latest_detection_poses = detections_msg.poses;
        _latest_detection_regions_of_interest = detections_msg.regions_of_interest;
        _latest_detection_bounding_boxes = std::move(bounding_boxes);
        _latest_detection_message = std::move(message);
    }

    void CubeDetector::_publish_depth_markers(
        const std::vector<detection_t>& detections,
        const std_msgs::msg::Header& header)
    {
        std::string source_frame;
        builtin_interfaces::msg::Time source_stamp;
        _get_depth_source(header, source_frame, source_stamp);

        std::string target_frame = _get_tf_output_frame();
        if (target_frame.empty())
        {
            target_frame = source_frame;
        }

        std_msgs::msg::Header marker_header = header;
        marker_header.frame_id = target_frame;
        marker_header.stamp = source_stamp;

        perseus_interfaces::msg::ObjectDetections detections_msg;
        detections_msg.stamp = source_stamp;
        detections_msg.frame_id = target_frame;

        visualization_msgs::msg::MarkerArray marker_array;
        marker_array.markers.push_back(make_clear_marker(marker_header));

        std::vector<cv::Rect> resolved_bounding_boxes;
        int32_t marker_id = 1;
        for (const auto& detection : detections)
        {
            geometry_msgs::msg::Pose pose;
            double distance_m = 0.0;
            if (!_estimate_cube_pose_from_depth(detection, pose, distance_m))
            {
                continue;
            }

            geometry_msgs::msg::Pose marker_pose = pose;
            if (target_frame != source_frame &&
                !_transform_pose_to_output_frame(pose, target_frame, source_frame, source_stamp, marker_pose))
            {
                continue;
            }

            marker_array.markers.push_back(
                make_cube_marker(marker_header, marker_id++, marker_pose, detection.class_id));
            marker_array.markers.push_back(
                make_text_marker(marker_header, marker_id++, marker_pose, detection.class_id));

            // Cube IDs are mapped by class order: blue=1, green=2, red=3, white=4.
            detections_msg.ids.push_back(detection.class_id + 1);
            detections_msg.poses.push_back(marker_pose);
            detections_msg.regions_of_interest.push_back(region_of_interest_from_rect(detection.bounding_box));
            resolved_bounding_boxes.push_back(detection.bounding_box);
        }

        _cube_marker_publisher->publish(marker_array);
        _cube_detection_publisher->publish(detections_msg);

        // Cache only detections that produced valid depth-derived poses.
        _cache_latest_detections(detections_msg,
                                 std::move(resolved_bounding_boxes),
                                 _build_depth_summary_message(detections.size(), detections_msg.ids.size()));
    }

    void CubeDetector::_publish_2d_detections(
        const std::vector<detection_t>& detections,
        const std_msgs::msg::Header& header)
    {
        perseus_interfaces::msg::ObjectDetections detections_msg;
        detections_msg.stamp = header.stamp;
        detections_msg.frame_id = header.frame_id;

        std::vector<cv::Rect> bounding_boxes;
        bounding_boxes.reserve(detections.size());

        for (const auto& detection : detections)
        {
            detections_msg.ids.push_back(detection.class_id + 1);
            detections_msg.poses.push_back(make_unresolved_pose());
            detections_msg.regions_of_interest.push_back(region_of_interest_from_rect(detection.bounding_box));
            bounding_boxes.push_back(detection.bounding_box);
        }

        _cube_detection_publisher->publish(detections_msg);

        std::string message;
        if (detections.empty())
        {
            message = "No cube detections found in the latest image.";
        }
        else
        {
            std::ostringstream message_stream;
            message_stream << "Detected " << detections.size()
                           << " cube(s) in 2D only; 3D poses are unavailable in the current depth mode.";
            message = message_stream.str();
        }

        _cache_latest_detections(detections_msg, std::move(bounding_boxes), std::move(message));
    }

    bool CubeDetector::_estimate_cube_pose_from_depth(
        const detection_t& detection,
        geometry_msgs::msg::Pose& pose_out,
        double& distance_m_out) const
    {
        sensor_msgs::msg::Image::SharedPtr depth_msg;
        double fx = 0.0;
        double fy = 0.0;
        double cx = 0.0;
        double cy = 0.0;

        {
            std::lock_guard<std::mutex> lock(_depth_mutex);
            if (!_latest_depth_image || !_has_depth_intrinsics)
            {
                return false;
            }
            depth_msg = _latest_depth_image;
            fx = _depth_fx;
            fy = _depth_fy;
            cx = _depth_cx;
            cy = _depth_cy;
        }

        if (!depth_msg || fx <= 0.0 || fy <= 0.0)
        {
            return false;
        }

        const int u = detection.bounding_box.x + (detection.bounding_box.width / 2);
        const int v = detection.bounding_box.y + (detection.bounding_box.height / 2);
        if (u < 0 || v < 0 || u >= static_cast<int>(depth_msg->width) || v >= static_cast<int>(depth_msg->height))
        {
            return false;
        }

        double depth_m = 0.0;
        if (!_read_depth_meters_at_pixel(*depth_msg, u, v, depth_m))
        {
            return false;
        }
        if (depth_m < _depth_min_range_m || depth_m > _depth_max_range_m)
        {
            return false;
        }

        const double x = (static_cast<double>(u) - cx) * depth_m / fx;
        const double y = (static_cast<double>(v) - cy) * depth_m / fy;
        const double z = depth_m;

        const double yaw = std::atan2(x, z);
        const double pitch = -std::atan2(y, std::sqrt((x * x) + (z * z)));

        pose_out.position.x = x;
        pose_out.position.y = y;
        pose_out.position.z = z;
        pose_out.orientation = quaternion_from_yaw_pitch(yaw, pitch);
        distance_m_out = std::sqrt((x * x) + (y * y) + (z * z));
        return true;
    }

    bool CubeDetector::_read_depth_meters_at_pixel(
        const sensor_msgs::msg::Image& depth_msg,
        int u,
        int v,
        double& depth_m_out) const
    {
        if (depth_msg.encoding != DEPTH_ENCODING_16UC1 && depth_msg.encoding != DEPTH_ENCODING_32FC1)
        {
            return false;
        }

        constexpr int samples_per_side = (2 * DEPTH_SAMPLE_RADIUS_PX) + 1;
        std::vector<double> samples;
        samples.reserve(samples_per_side * samples_per_side);

        for (int row_offset = -DEPTH_SAMPLE_RADIUS_PX; row_offset <= DEPTH_SAMPLE_RADIUS_PX; ++row_offset)
        {
            const int row = v + row_offset;
            if (row < 0 || row >= static_cast<int>(depth_msg.height))
            {
                continue;
            }

            for (int column_offset = -DEPTH_SAMPLE_RADIUS_PX; column_offset <= DEPTH_SAMPLE_RADIUS_PX; ++column_offset)
            {
                const int column = u + column_offset;
                if (column < 0 || column >= static_cast<int>(depth_msg.width))
                {
                    continue;
                }

                const double depth_m = _read_depth_sample(depth_msg, row, column);
                if (std::isfinite(depth_m))
                {
                    samples.push_back(depth_m);
                }
            }
        }

        if (samples.empty())
        {
            return false;
        }

        std::sort(samples.begin(), samples.end());
        depth_m_out = samples[samples.size() / 2];
        return true;
    }

    double CubeDetector::_read_depth_sample(const sensor_msgs::msg::Image& depth_msg, int row, int column) const
    {
        if (depth_msg.encoding == DEPTH_ENCODING_16UC1)
        {
            const auto* row_data = reinterpret_cast<const uint16_t*>(
                &depth_msg.data[static_cast<std::size_t>(row) * depth_msg.step]);
            const uint16_t raw_depth = row_data[column];
            if (raw_depth == 0)
            {
                return std::numeric_limits<double>::quiet_NaN();
            }
            return static_cast<double>(raw_depth) * _depth_unit_scale;
        }

        const auto* row_data = reinterpret_cast<const float*>(
            &depth_msg.data[static_cast<std::size_t>(row) * depth_msg.step]);
        return static_cast<double>(row_data[column]);
    }

    bool CubeDetector::_transform_pose_to_output_frame(
        const geometry_msgs::msg::Pose& input_pose,
        const std::string& target_frame,
        const std::string& source_frame,
        const builtin_interfaces::msg::Time& stamp,
        geometry_msgs::msg::Pose& output_pose) const
    {
        if (!_tf_buffer)
        {
            return false;
        }

        geometry_msgs::msg::PoseStamped in_pose;
        in_pose.header.frame_id = source_frame;
        in_pose.header.stamp = stamp;
        in_pose.pose = input_pose;

        try
        {
            const geometry_msgs::msg::PoseStamped out_pose = _tf_buffer->transform(in_pose, target_frame);
            output_pose = out_pose.pose;
            return true;
        }
        catch (const tf2::TransformException& ex)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                LOG_THROTTLE_MS,
                "Failed to transform cube pose from %s to %s: %s",
                source_frame.c_str(),
                target_frame.c_str(),
                ex.what());
            return false;
        }
    }

    void CubeDetector::_handle_detect_objects_request(
        const std::shared_ptr<DetectObjects::Request> request,
        std::shared_ptr<DetectObjects::Response> response)
    {
        if (!_is_always_on.load())
        {
            _run_inference_on_cached_frame();
        }

        cv::Mat frame_to_save;
        std::vector<int32_t> ids;
        std::vector<geometry_msgs::msg::Pose> poses;
        std::vector<cv::Rect> bounding_boxes;
        std::string frame_id;

        {
            std::lock_guard<std::mutex> lock(_detections_mutex);
            response->stamp = _latest_detections_stamp;
            response->frame_id = _latest_detections_frame_id;
            response->ids = _latest_detection_ids;
            response->poses = _latest_detection_poses;
            response->regions_of_interest = _latest_detection_regions_of_interest;
            response->message = _latest_detection_message;

            if (request->capture_image)
            {
                frame_to_save = _latest_annotated_frame.clone();
                ids = _latest_detection_ids;
                poses = _latest_detection_poses;
                bounding_boxes = _latest_detection_bounding_boxes;
                frame_id = _latest_detections_frame_id;
            }
        }

        if (!request->capture_image)
        {
            return;
        }

        _save_detection_capture(request->img_save_path, frame_to_save, ids, poses, bounding_boxes, frame_id);
    }

    void CubeDetector::_run_inference_on_cached_frame()
    {
        cv::Mat latest_frame;
        std_msgs::msg::Header latest_header;

        {
            std::lock_guard<std::mutex> lock(_latest_image_mutex);
            if (!_has_latest_bgr_frame || _latest_bgr_frame.empty())
            {
                RCLCPP_WARN(get_logger(), "detect_objects requested, but no camera frame has been received yet");
                return;
            }
            latest_frame = _latest_bgr_frame.clone();
            latest_header = _latest_bgr_header;
        }

        std::lock_guard<std::mutex> lock(_inference_mutex);
        _run_inference_pipeline(latest_frame, latest_header);
    }

    void CubeDetector::_save_detection_capture(
        const std::string& save_path,
        cv::Mat& frame,
        const std::vector<int32_t>& ids,
        const std::vector<geometry_msgs::msg::Pose>& poses,
        const std::vector<cv::Rect>& bounding_boxes,
        const std::string& frame_id) const
    {
        if (frame.empty())
        {
            RCLCPP_WARN(get_logger(), "detect_objects capture requested, but no annotated frame available");
            return;
        }

        const std::string save_directory = save_path.empty() ? "." : save_path;

        std::error_code error_code;
        std::filesystem::create_directories(save_directory, error_code);
        if (error_code)
        {
            RCLCPP_WARN(get_logger(), "Failed to create directory %s: %s",
                        save_directory.c_str(), error_code.message().c_str());
        }

        draw_detection_overlay(frame, ids, poses, bounding_boxes, frame_id);

        const auto epoch_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                  std::chrono::system_clock::now().time_since_epoch())
                                  .count();
        const std::string filename = save_directory + "/cube_detections_" + std::to_string(epoch_ms) + ".png";
        if (!cv::imwrite(filename, frame))
        {
            RCLCPP_ERROR(get_logger(), "Failed to save capture image to %s", filename.c_str());
            return;
        }

        RCLCPP_INFO(get_logger(), "Saved cube detection capture: %s", filename.c_str());
    }

    rcl_interfaces::msg::SetParametersResult CubeDetector::_parameter_callback(
        const std::vector<rclcpp::Parameter>& parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        for (const auto& parameter : parameters)
        {
            if (parameter.get_name() == "always_on")
            {
                _is_always_on.store(parameter.as_bool());
                continue;
            }

            if (parameter.get_name() == "processing_frequency_hz")
            {
                const double processing_frequency_hz = parameter.as_double();
                if (processing_frequency_hz < 0.0)
                {
                    result.successful = false;
                    result.reason = "processing_frequency_hz must be non-negative";
                    return result;
                }

                _processing_frequency_hz.store(processing_frequency_hz);
                continue;
            }

            if (parameter.get_name() == "tf_output_frame")
            {
                std::lock_guard<std::mutex> lock(_parameter_mutex);
                _tf_output_frame = parameter.as_string();
            }
        }

        return result;
    }

}  // namespace perseus_vision
