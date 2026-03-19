#include "perseus_vision/cube_detector.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace perseus_vision
{

    // ── constructor ───────────────────────────────────────────────────────────────
    CubeDetector::CubeDetector()
        : Node("cube_detector"),
          _ort_env(ORT_LOGGING_LEVEL_WARNING, "cube_detector")
    {
        // ── parameters ──────────────────────────────────────────────────────────────
        declare_parameter("model_path", "");
        declare_parameter("confidence_threshold", 0.3);
        declare_parameter("camera_topic", "/camera/camera/color/image_raw");
        declare_parameter("camera_info_topic", "/camera/camera/color/camera_info");
        declare_parameter("always_on", true);  // keep node alive even without subscribers
        declare_parameter("use_cuda", false);
        declare_parameter("publish_annotated_image", true);
        declare_parameter("processing_frequency_hz", 0.0);  // 0 = process every frame
        declare_parameter("intra_op_num_threads", 4);
        declare_parameter("inter_op_num_threads", 2);
        declare_parameter("nms_iou_threshold", 0.45);
        declare_parameter("depth_estimation_mode", "none");
        declare_parameter("depth_image.topic", "/camera/camera/depth/image_rect_raw");
        declare_parameter("depth_image.info_topic", "/camera/camera/depth/camera_info");
        declare_parameter("depth_image.unit_scale", 0.001);
        declare_parameter("depth_image.max_range_m", 5.0);
        declare_parameter("depth_image.min_range_m", 0.1);
        declare_parameter("tf_output_frame", "odom");
        declare_parameter("output_img_topic", "/perseus_vision/cube/image_annotated");
        declare_parameter("output_detections_topic", "/perseus_vision/cube/detections");
        declare_parameter("output_markers_topic", "/perseus_vision/cube/markers");
        const auto camera_topic = get_parameter("camera_topic").as_string();
        const auto camera_info_topic = get_parameter("camera_info_topic").as_string();
        _intra_op_num_threads = get_parameter("intra_op_num_threads").as_int();
        _inter_op_num_threads = get_parameter("inter_op_num_threads").as_int();
        _nms_iou_threshold = static_cast<float>(get_parameter("nms_iou_threshold").as_double());
        _depth_estimation_mode = get_parameter("depth_estimation_mode").as_string();
        _depth_topic = get_parameter("depth_image.topic").as_string();
        _depth_info_topic = get_parameter("depth_image.info_topic").as_string();
        _depth_unit_scale = get_parameter("depth_image.unit_scale").as_double();
        _depth_max_range_m = get_parameter("depth_image.max_range_m").as_double();
        _depth_min_range_m = get_parameter("depth_image.min_range_m").as_double();
        _confidence_threshold = static_cast<float>(get_parameter("confidence_threshold").as_double());
        _always_on.store(get_parameter("always_on").as_bool());  // set initial value of atomic bool
        _should_use_cuda = get_parameter("use_cuda").as_bool();
        _publish_annotated_image = get_parameter("publish_annotated_image").as_bool();
        _processing_frequency_hz.store(get_parameter("processing_frequency_hz").as_double());
        _model_path = get_parameter("model_path").as_string();
        _tf_output_frame = get_parameter("tf_output_frame").as_string();
        _output_img_topic = get_parameter("output_img_topic").as_string();
        _output_detections_topic = get_parameter("output_detections_topic").as_string();
        _output_markers_topic = get_parameter("output_markers_topic").as_string();
        _param_callback_handle = add_on_set_parameters_callback(
            std::bind(&CubeDetector::parameter_callback, this, std::placeholders::_1));
        if (_model_path.rfind("~/", 0) == 0)
        {
            if (const char* home = std::getenv("HOME"))
            {
                _model_path = std::string(home) + _model_path.substr(1);
            }
        }

        // Backward compatibility: some configs use "/perseus-v2/..." (missing $HOME prefix).
        if (!_model_path.empty() && !std::filesystem::exists(_model_path))
        {
            if (const char* home = std::getenv("HOME"))
            {
                const std::string home_prefixed = std::string(home) + _model_path;
                if (std::filesystem::exists(home_prefixed))
                {
                    _model_path = home_prefixed;
                }
            }
        }

        if (_model_path.empty() || !std::filesystem::exists(_model_path))
        {
            const std::string packaged_model =
                ament_index_cpp::get_package_share_directory("perseus_vision") + "/models/cube_detector_yolob8s.onnx";
            if (std::filesystem::exists(packaged_model))
            {
                _model_path = packaged_model;
            }
        }

        // ── load ONNX model ──────────────────────────────────────────────────────────
        RCLCPP_INFO(get_logger(), "Loading model: %s", _model_path.c_str());

        Ort::SessionOptions session_opts;
        session_opts.SetIntraOpNumThreads(_intra_op_num_threads);
        session_opts.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);
        session_opts.SetInterOpNumThreads(_inter_op_num_threads);

        if (_should_use_cuda)
        {
            OrtCUDAProviderOptions cuda_opts{};
            try
            {
                session_opts.AppendExecutionProvider_CUDA(cuda_opts);
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
                _ort_env, _model_path.c_str(), session_opts);
        }
        catch (const Ort::Exception& e)
        {
            RCLCPP_FATAL(get_logger(), "Failed to load ONNX model from '%s': %s", _model_path.c_str(), e.what());
            throw;
        }

        auto input_name = _ort_session->GetInputNameAllocated(0, _ort_allocator);
        auto output_name = _ort_session->GetOutputNameAllocated(0, _ort_allocator);
        _input_name = input_name.get();
        _output_name = output_name.get();

        RCLCPP_INFO(get_logger(), "Model loaded — input: %s  output: %s",
                    _input_name.c_str(), _output_name.c_str());

        // ── subscribers ──────────────────────────────────────────────────────────────
        _sub_image = create_subscription<sensor_msgs::msg::Image>(
            camera_topic, kQosDepth,
            std::bind(&CubeDetector::image_callback, this, std::placeholders::_1));

        _sub_camera_info = create_subscription<sensor_msgs::msg::CameraInfo>(
            camera_info_topic, kQosDepth,
            std::bind(&CubeDetector::camera_info_callback, this, std::placeholders::_1));
        if (_depth_estimation_mode == "depth_image")
        {
            _sub_depth_image = create_subscription<sensor_msgs::msg::Image>(
                _depth_topic, kQosDepth,
                std::bind(&CubeDetector::depth_image_callback, this, std::placeholders::_1));
            _sub_depth_camera_info = create_subscription<sensor_msgs::msg::CameraInfo>(
                _depth_info_topic, kQosDepth,
                std::bind(&CubeDetector::depth_camera_info_callback, this, std::placeholders::_1));
        }

        // ── publishers ───────────────────────────────────────────────────────────────
        _pub_annotated = create_publisher<sensor_msgs::msg::Image>(
            _output_img_topic, kQosDepth);
        _pub_cube_markers = create_publisher<visualization_msgs::msg::MarkerArray>(
            _output_markers_topic, kQosDepth);
        _pub_cube_detections = create_publisher<perseus_interfaces::msg::ObjectDetections>(
            _output_detections_topic, kQosDepth);
        _srv_detect_objects = create_service<DetectObjects>(
            "detect_objects",
            std::bind(&CubeDetector::handle_detect_objects_request, this, std::placeholders::_1, std::placeholders::_2));
        _tf_buffer = std::make_unique<tf2_ros::Buffer>(get_clock());
        _tf_listener = std::make_shared<tf2_ros::TransformListener>(*_tf_buffer);

        RCLCPP_INFO(get_logger(), "CubeDetectorNode ready — subscribed to %s", camera_topic.c_str());
        if (_always_on.load())
        {
            RCLCPP_INFO(get_logger(), "Node is set to always_on, will process images even without subscribers");
        }
        else
        {
            RCLCPP_INFO(get_logger(), "Always on is disabled!");
        }
    }

    // ── image callback ────────────────────────────────────────────────────────────
    void CubeDetector::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (const cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        // Always cache latest frame so service can run on-demand even when always_on is false.
        {
            std::lock_guard<std::mutex> lock(_latest_image_mutex);
            _latest_bgr_frame = cv_ptr->image.clone();
            _latest_bgr_header = msg->header;
            _has_latest_bgr_frame = true;
        }

        if (!_always_on.load())
        {
            return;
        }

        const double processing_frequency_hz = _processing_frequency_hz.load();
        if (processing_frequency_hz > 0.0)
        {
            const int64_t now_ns = this->now().nanoseconds();
            if (_last_inference_time_ns != 0 && now_ns < _last_inference_time_ns)
            {
                _last_inference_time_ns = 0;
            }
            const int64_t min_period_ns = static_cast<int64_t>(1e9 / processing_frequency_hz);
            if (_last_inference_time_ns != 0 && (now_ns - _last_inference_time_ns) < min_period_ns)
            {
                return;
            }
            _last_inference_time_ns = now_ns;
        }

        std::lock_guard<std::mutex> lock(_inference_mutex);
        run_inference_pipeline(cv_ptr->image, msg->header);
    }

    // ── camera info callback (kept for future depth use) ─────────────────────────
    void CubeDetector::camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(_camera_matrix_mutex);

        _camera_matrix = cv::Mat(3, 3, CV_64F);
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                _camera_matrix.at<double>(r, c) = msg->k[r * 3 + c];

        _dist_coeffs = cv::Mat(static_cast<int>(msg->d.size()), 1, CV_64F);
        for (size_t i = 0; i < msg->d.size(); ++i)
            _dist_coeffs.at<double>(static_cast<int>(i), 0) = msg->d[i];
    }

    void CubeDetector::depth_image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(_depth_mutex);
        _latest_depth_image = msg;
    }

    void CubeDetector::depth_camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(_depth_mutex);
        _depth_fx = msg->k[0];
        _depth_fy = msg->k[4];
        _depth_cx = msg->k[2];
        _depth_cy = msg->k[5];
        _has_depth_intrinsics = _depth_fx > 0.0 && _depth_fy > 0.0;
    }

    // ── preprocess ────────────────────────────────────────────────────────────────
    std::vector<float> CubeDetector::preprocess(const cv::Mat& bgr_image)
    {
        _orig_h = bgr_image.rows;
        _orig_w = bgr_image.cols;

        // ── letterbox (preserve aspect ratio) ───────────────────────────────
        float scale = std::min(640.0f / _orig_w, 640.0f / _orig_h);
        int new_w = static_cast<int>(_orig_w * scale);
        int new_h = static_cast<int>(_orig_h * scale);

        _pad_x = (640 - new_w) / 2;
        _pad_y = (640 - new_h) / 2;
        _letterbox_scale = scale;

        cv::Mat resized;
        cv::resize(bgr_image, resized, cv::Size(new_w, new_h));

        // pad to 640x640 with grey (114 = YOLOv8 default padding value)
        cv::Mat padded(640, 640, CV_8UC3, cv::Scalar(114, 114, 114));
        resized.copyTo(padded(cv::Rect(_pad_x, _pad_y, new_w, new_h)));

        // ── BGR → RGB → float → CHW ─────────────────────────────────────────
        cv::Mat rgb;
        cv::cvtColor(padded, rgb, cv::COLOR_BGR2RGB);
        rgb.convertTo(rgb, CV_32FC3, 1.0 / 255.0);

        std::vector<cv::Mat> channels(3);
        cv::split(rgb, channels);

        std::vector<float> blob;
        blob.reserve(3 * 640 * 640);
        for (auto& ch : channels)
            blob.insert(blob.end(),
                        reinterpret_cast<float*>(ch.data),
                        reinterpret_cast<float*>(ch.data) + 640 * 640);
        return blob;
    }
    // ── postprocess ───────────────────────────────────────────────────────────────
    std::vector<Detection> CubeDetector::postprocess(const float* data, size_t num_boxes)
    {
        std::vector<cv::Rect> boxes;
        std::vector<float> scores;
        std::vector<int> class_ids;

        for (size_t i = 0; i < num_boxes; ++i)
        {
            const float xc = data[0 * num_boxes + i];
            const float yc = data[1 * num_boxes + i];
            const float w = data[2 * num_boxes + i];
            const float h = data[3 * num_boxes + i];

            int best_class = 0;
            float best_score = 0.0f;
            for (int c = 0; c < _num_classes; ++c)
            {
                float score = data[(4 + c) * num_boxes + i];
                if (score > best_score)
                {
                    best_score = score;
                    best_class = c;
                }
            }

            if (best_score < _confidence_threshold)
                continue;
            // YOLOv8 outputs box coordinates in the letterboxed 640x640 space, so we need to reverse the letterbox transformation to get back to original image coordinates
            const int x1 = std::clamp(static_cast<int>((xc - w / 2.0f - _pad_x) / _letterbox_scale), 0, _orig_w);
            const int y1 = std::clamp(static_cast<int>((yc - h / 2.0f - _pad_y) / _letterbox_scale), 0, _orig_h);
            const int x2 = std::clamp(static_cast<int>((xc + w / 2.0f - _pad_x) / _letterbox_scale), 0, _orig_w);
            const int y2 = std::clamp(static_cast<int>((yc + h / 2.0f - _pad_y) / _letterbox_scale), 0, _orig_h);

            boxes.push_back(cv::Rect(x1, y1, x2 - x1, y2 - y1));
            scores.push_back(best_score);
            class_ids.push_back(best_class);
        }

        // NMS — guard against empty boxes
        std::vector<int> indices;
        if (!boxes.empty())
            // Suppress non-maximal boxes with IOU > 0.45 (YOLOv8 default NMS threshold)
            cv::dnn::NMSBoxes(boxes, scores, _confidence_threshold, _nms_iou_threshold, indices);

        std::vector<Detection> detections;
        for (int idx : indices)
            detections.push_back({class_ids[idx], scores[idx], boxes[idx]});

        return detections;
    }

    // ── publish annotated image ───────────────────────────────────────────────────
    void CubeDetector::publish_annotated_image(
        const cv::Mat& image,
        const std::vector<Detection>& detections,
        const std_msgs::msg::Header& header)
    {
        cv::Mat annotated = image.clone();

        for (const auto& d : detections)
        {
            const cv::Scalar& colour = CLASS_COLOURS[d.class_id];

            // bounding box
            cv::rectangle(annotated, d.bbox, colour, 2);

            // label: "blue 0.97"
            const std::string label = CLASS_NAMES[d.class_id] + " " + std::to_string(d.confidence).substr(0, 4);

            // center coordinates label: "cx:320 cy:240"
            const int cx = d.bbox.x + d.bbox.width / 2;
            const int cy = d.bbox.y + d.bbox.height / 2;
            const std::string center_label = "cx:" + std::to_string(cx) + " cy:" + std::to_string(cy);

            // draw class + confidence above box
            cv::putText(annotated, label,
                        cv::Point(d.bbox.x, d.bbox.y - 20),
                        cv::FONT_HERSHEY_SIMPLEX, 0.6, colour, 2);

            // draw center coordinates below class label
            cv::putText(annotated, center_label,
                        cv::Point(d.bbox.x, d.bbox.y - 5),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, colour, 1);

            // draw crosshair at center
            cv::drawMarker(annotated, cv::Point(cx, cy), colour,
                           cv::MARKER_CROSS, 15, 1);
        }

        if (_publish_annotated_image)
        {
            _pub_annotated->publish(
                *cv_bridge::CvImage(header, "bgr8", annotated).toImageMsg());
        }
        {
            std::lock_guard<std::mutex> lock(_detections_mutex);
            _latest_annotated_frame = annotated.clone();
        }
    }

    void CubeDetector::publish_depth_markers(
        const std::vector<Detection>& detections,
        const std_msgs::msg::Header& header)
    {
        visualization_msgs::msg::MarkerArray marker_array;
        std_msgs::msg::Header marker_header = header;
        std::string source_frame = header.frame_id;
        builtin_interfaces::msg::Time source_stamp = header.stamp;
        {
            std::lock_guard<std::mutex> lock(_depth_mutex);
            if (_latest_depth_image)
            {
                source_frame = _latest_depth_image->header.frame_id;
                source_stamp = _latest_depth_image->header.stamp;
            }
        }
        _tf_output_frame = get_parameter("tf_output_frame").as_string();
        if (_tf_output_frame.empty())
        {
            _tf_output_frame = source_frame;
        }
        marker_header.frame_id = _tf_output_frame;
        marker_header.stamp = source_stamp;
        perseus_interfaces::msg::ObjectDetections detections_msg;
        detections_msg.stamp = source_stamp;
        detections_msg.frame_id = marker_header.frame_id;
        std::vector<cv::Rect> detection_bboxes_for_service;
        std::string detection_message;

        visualization_msgs::msg::Marker clear_marker;
        clear_marker.header = marker_header;
        clear_marker.ns = "cube_depth";
        clear_marker.id = 0;
        clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array.markers.push_back(clear_marker);

        int marker_id = 1;
        for (const auto& detection : detections)
        {
            geometry_msgs::msg::Pose pose;
            double distance_m = 0.0;
            if (!estimate_cube_pose_from_depth(detection, pose, distance_m))
            {
                continue;
            }
            geometry_msgs::msg::Pose marker_pose = pose;
            if (_tf_output_frame != source_frame)
            {
                if (!transform_pose_to_output_frame(pose, source_frame, source_stamp, marker_pose))
                {
                    continue;
                }
            }

            visualization_msgs::msg::Marker cube_marker;
            cube_marker.header = marker_header;
            cube_marker.ns = "cube_depth";
            cube_marker.id = marker_id++;
            cube_marker.type = visualization_msgs::msg::Marker::CUBE;
            cube_marker.action = visualization_msgs::msg::Marker::ADD;
            cube_marker.pose = marker_pose;
            cube_marker.scale.x = 0.12;
            cube_marker.scale.y = 0.12;
            cube_marker.scale.z = 0.12;
            cube_marker.color.a = 0.85f;
            cube_marker.color.r = static_cast<float>(CLASS_COLOURS[detection.class_id][2] / 255.0);
            cube_marker.color.g = static_cast<float>(CLASS_COLOURS[detection.class_id][1] / 255.0);
            cube_marker.color.b = static_cast<float>(CLASS_COLOURS[detection.class_id][0] / 255.0);
            cube_marker.lifetime = rclcpp::Duration::from_seconds(0.2);
            marker_array.markers.push_back(cube_marker);

            visualization_msgs::msg::Marker text_marker;
            text_marker.header = marker_header;
            text_marker.ns = "cube_depth_text";
            text_marker.id = marker_id++;
            text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            text_marker.action = visualization_msgs::msg::Marker::ADD;
            text_marker.pose = marker_pose;
            text_marker.pose.position.z += 0.12;
            text_marker.scale.z = 0.08;
            text_marker.color.a = 1.0f;
            text_marker.color.r = 1.0f;
            text_marker.color.g = 1.0f;
            text_marker.color.b = 1.0f;
            std::ostringstream ss;
            ss << CLASS_NAMES[detection.class_id]
               << " x:" << std::fixed << std::setprecision(2) << marker_pose.position.x
               << " y:" << std::fixed << std::setprecision(2) << marker_pose.position.y;
            text_marker.text = ss.str();
            text_marker.lifetime = rclcpp::Duration::from_seconds(0.2);
            marker_array.markers.push_back(text_marker);

            // Cube IDs are mapped by class order: blue=1, green=2, red=3, white=4.
            detections_msg.ids.push_back(detection.class_id + 1);
            detections_msg.poses.push_back(marker_pose);
            detection_bboxes_for_service.push_back(detection.bbox);
        }

        if (detections.empty())
        {
            detection_message = "No cube detections found in the latest image.";
        }
        else if (detections_msg.ids.empty())
        {
            std::ostringstream message_stream;
            message_stream << "Detected " << detections.size()
                           << " cube(s) in 2D, but depth estimation failed for all detections. "
                           << "Returning 0 depth-resolved cube pose(s).";
            detection_message = message_stream.str();
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "%s",
                detection_message.c_str());
        }
        else if (detections_msg.ids.size() < detections.size())
        {
            std::ostringstream message_stream;
            message_stream << "Resolved " << detections_msg.ids.size() << " of " << detections.size()
                           << " detected cube(s) to depth-derived poses.";
            detection_message = message_stream.str();
        }
        else
        {
            std::ostringstream message_stream;
            message_stream << "Resolved all " << detections_msg.ids.size()
                           << " detected cube(s) to depth-derived poses.";
            detection_message = message_stream.str();
        }

        _pub_cube_markers->publish(marker_array);
        _pub_cube_detections->publish(detections_msg);
        {
            std::lock_guard<std::mutex> lock(_detections_mutex);
            // Cache only detections that produced valid depth-derived poses.
            _latest_detections_stamp = detections_msg.stamp;
            _latest_detections_frame_id = detections_msg.frame_id;
            _latest_detection_ids = detections_msg.ids;
            _latest_detection_poses = detections_msg.poses;
            _latest_detection_bboxes = std::move(detection_bboxes_for_service);
            _latest_detection_message = std::move(detection_message);
        }
    }

    bool CubeDetector::estimate_cube_pose_from_depth(
        const Detection& detection,
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

        const int u = detection.bbox.x + detection.bbox.width / 2;
        const int v = detection.bbox.y + detection.bbox.height / 2;
        if (u < 0 || v < 0 || u >= static_cast<int>(depth_msg->width) || v >= static_cast<int>(depth_msg->height))
        {
            return false;
        }

        double depth_m = 0.0;
        if (!read_depth_meters_at_pixel(*depth_msg, u, v, depth_m))
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
        const double pitch = -std::atan2(y, std::sqrt(x * x + z * z));

        pose_out.position.x = x;
        pose_out.position.y = y;
        pose_out.position.z = z;
        pose_out.orientation = quaternion_from_yaw_pitch(yaw, pitch);
        distance_m_out = std::sqrt(x * x + y * y + z * z);
        return true;
    }

    bool CubeDetector::read_depth_meters_at_pixel(
        const sensor_msgs::msg::Image& depth_msg,
        int u,
        int v,
        double& depth_m_out) const
    {
        const int radius = 1;
        std::vector<double> samples;
        samples.reserve((2 * radius + 1) * (2 * radius + 1));

        for (int dv = -radius; dv <= radius; ++dv)
        {
            const int yy = v + dv;
            if (yy < 0 || yy >= static_cast<int>(depth_msg.height))
            {
                continue;
            }
            for (int du = -radius; du <= radius; ++du)
            {
                const int xx = u + du;
                if (xx < 0 || xx >= static_cast<int>(depth_msg.width))
                {
                    continue;
                }

                double depth_m = std::numeric_limits<double>::quiet_NaN();
                if (depth_msg.encoding == "16UC1")
                {
                    const auto* row = reinterpret_cast<const uint16_t*>(&depth_msg.data[yy * depth_msg.step]);
                    const uint16_t raw = row[xx];
                    if (raw > 0)
                    {
                        depth_m = static_cast<double>(raw) * _depth_unit_scale;
                    }
                }
                else if (depth_msg.encoding == "32FC1")
                {
                    const auto* row = reinterpret_cast<const float*>(&depth_msg.data[yy * depth_msg.step]);
                    depth_m = static_cast<double>(row[xx]);
                }
                else
                {
                    return false;
                }

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

    geometry_msgs::msg::Quaternion CubeDetector::quaternion_from_yaw_pitch(double yaw, double pitch) const
    {
        const double half_yaw = 0.5 * yaw;
        const double half_pitch = 0.5 * pitch;
        const double cy = std::cos(half_yaw);
        const double sy = std::sin(half_yaw);
        const double cp = std::cos(half_pitch);
        const double sp = std::sin(half_pitch);

        geometry_msgs::msg::Quaternion q;
        q.w = cy * cp;
        q.x = -sy * sp;
        q.y = cy * sp;
        q.z = sy * cp;
        return q;
    }

    bool CubeDetector::transform_pose_to_output_frame(
        const geometry_msgs::msg::Pose& input_pose,
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
            const geometry_msgs::msg::PoseStamped out_pose = _tf_buffer->transform(in_pose, _tf_output_frame);
            output_pose = out_pose.pose;
            return true;
        }
        catch (const tf2::TransformException& ex)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "Failed to transform cube pose from %s to %s: %s",
                source_frame.c_str(),
                _tf_output_frame.c_str(),
                ex.what());
            return false;
        }
    }

    void CubeDetector::run_inference_pipeline(
        const cv::Mat& bgr_image,
        const std_msgs::msg::Header& header)
    {
        auto blob = preprocess(bgr_image);

        std::array<int64_t, 4> input_shape = {1, 3, 640, 640};
        Ort::MemoryInfo mem_info =
            Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);

        Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
            mem_info, blob.data(), blob.size(),
            input_shape.data(), input_shape.size());

        const char* input_names[] = {_input_name.c_str()};
        const char* output_names[] = {_output_name.c_str()};

        auto output_tensors = _ort_session->Run(
            Ort::RunOptions{nullptr},
            input_names, &input_tensor, 1,
            output_names, 1);

        const float* out_data = output_tensors[0].GetTensorData<float>();
        const auto out_shape = output_tensors[0].GetTensorTypeAndShapeInfo().GetShape();
        const size_t num_boxes = static_cast<size_t>(out_shape[2]);

        auto detections = postprocess(out_data, num_boxes);
        publish_annotated_image(bgr_image, detections, header);

        if (_depth_estimation_mode == "depth_image")
        {
            publish_depth_markers(detections, header);
        }
    }

    void CubeDetector::handle_detect_objects_request(
        const std::shared_ptr<DetectObjects::Request> request,
        std::shared_ptr<DetectObjects::Response> response)
    {
        if (!_always_on.load())
        {
            cv::Mat latest_frame;
            std_msgs::msg::Header latest_header;
            {
                std::lock_guard<std::mutex> lock(_latest_image_mutex);
                if (!_has_latest_bgr_frame || _latest_bgr_frame.empty())
                {
                    RCLCPP_WARN(get_logger(), "detect_objects requested, but no camera frame has been received yet");
                    response->message = "No camera frame has been received yet.";
                }
                else
                {
                    latest_frame = _latest_bgr_frame.clone();
                    latest_header = _latest_bgr_header;
                }
            }
            if (!latest_frame.empty())
            {
                std::lock_guard<std::mutex> lock(_inference_mutex);
                run_inference_pipeline(latest_frame, latest_header);
            }
        }

        cv::Mat frame_to_save;
        std::vector<int32_t> ids_copy;
        std::vector<geometry_msgs::msg::Pose> poses_copy;
        std::vector<cv::Rect> bboxes_copy;
        std::string frame_id_copy;

        {
            std::lock_guard<std::mutex> lock(_detections_mutex);
            response->stamp = _latest_detections_stamp;
            response->frame_id = _latest_detections_frame_id;
            response->ids = _latest_detection_ids;
            response->poses = _latest_detection_poses;
            response->message = _latest_detection_message;

            if (request->capture_image)
            {
                frame_to_save = _latest_annotated_frame.clone();
                ids_copy = _latest_detection_ids;
                poses_copy = _latest_detection_poses;
                bboxes_copy = _latest_detection_bboxes;
                frame_id_copy = _latest_detections_frame_id;
            }
        }

        if (!request->capture_image)
        {
            return;
        }

        if (frame_to_save.empty())
        {
            RCLCPP_WARN(get_logger(), "detect_objects capture requested, but no annotated frame available");
            return;
        }

        std::string save_dir = request->img_save_path;
        if (save_dir.empty())
        {
            save_dir = ".";
        }

        std::error_code ec;
        std::filesystem::create_directories(save_dir, ec);
        if (ec)
        {
            RCLCPP_WARN(get_logger(), "Failed to create directory %s: %s", save_dir.c_str(), ec.message().c_str());
        }

        auto now = std::chrono::system_clock::now();
        const auto epoch_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
        const auto now_time_t = std::chrono::system_clock::to_time_t(now);
        struct tm local_tm;
        localtime_r(&now_time_t, &local_tm);
        char timestamp_str[64];
        std::strftime(timestamp_str, sizeof(timestamp_str), "%Y-%m-%d %H:%M:%S", &local_tm);

        cv::putText(
            frame_to_save,
            std::string("Time: ") + timestamp_str,
            cv::Point(10, 25),
            cv::FONT_HERSHEY_SIMPLEX,
            0.6,
            cv::Scalar(0, 255, 255),
            2);

        int y = 50;
        cv::putText(
            frame_to_save,
            std::string("Frame: ") + frame_id_copy,
            cv::Point(10, y),
            cv::FONT_HERSHEY_SIMPLEX,
            0.5,
            cv::Scalar(255, 255, 0),
            1);

        for (size_t i = 0; i < ids_copy.size() && i < poses_copy.size(); ++i)
        {
            if (i < bboxes_copy.size())
            {
                cv::rectangle(frame_to_save, bboxes_copy[i], cv::Scalar(0, 255, 255), 2);
            }
            y += 20;
            std::ostringstream line;
            line << "ID " << ids_copy[i]
                 << " x=" << std::fixed << std::setprecision(2) << poses_copy[i].position.x
                 << " y=" << std::fixed << std::setprecision(2) << poses_copy[i].position.y;
            cv::putText(
                frame_to_save,
                line.str(),
                cv::Point(10, y),
                cv::FONT_HERSHEY_SIMPLEX,
                0.5,
                cv::Scalar(0, 255, 0),
                1);
        }

        const std::string filename = save_dir + "/cube_detections_" + std::to_string(epoch_ms) + ".png";
        if (!cv::imwrite(filename, frame_to_save))
        {
            RCLCPP_ERROR(get_logger(), "Failed to save capture image to %s", filename.c_str());
            return;
        }

        RCLCPP_INFO(get_logger(), "Saved cube detection capture: %s", filename.c_str());
    }

    rcl_interfaces::msg::SetParametersResult CubeDetector::parameter_callback(
        const std::vector<rclcpp::Parameter>& parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        for (const auto& parameter : parameters)
        {
            if (parameter.get_name() == "always_on")
            {
                _always_on.store(parameter.as_bool());
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
            }
        }

        return result;
    }

}  // namespace perseus_vision
