#include "perseus_vision/cube_detector.hpp"

#include <algorithm>
#include <string>
#include <vector>

namespace perseus_vision
{

    // ── constructor ───────────────────────────────────────────────────────────────
    CubeDetector::CubeDetector()
        : Node("cube_detector"),
          _ort_env(ORT_LOGGING_LEVEL_WARNING, "cube_detector")
    {
        // ── parameters ──────────────────────────────────────────────────────────────
        declare_parameter("model_path", "/perseus-v2/software/ros_ws/src/perseus_vision/models/cube_detector_yolob8s.onnx");
        declare_parameter("confidence_threshold", 0.5);
        declare_parameter("camera_topic", "/image_raw");
        declare_parameter("camera_info_topic", "/camera/camera/color/camera_info");
        declare_parameter("always_on", true);  // keep node alive even without subscribers
        declare_parameter("use_cuda", true);
        declare_parameter("publish_annotated_image", true);
        declare_parameter("processing_frequency_hz", 0.0);  // 0 = process every frame
        declare_parameter("intra_op_num_threads", 4);
        declare_parameter("inter_op_num_threads", 2);
        declare_parameter("nms_iou_threshold", 0.45);
        declare_parameter("depth_estimation_mode", "none");
        const auto camera_topic = get_parameter("camera_topic").as_string();
        const auto camera_info_topic = get_parameter("camera_info_topic").as_string();
        _intra_op_num_threads = get_parameter("intra_op_num_threads").as_int();
        _inter_op_num_threads = get_parameter("inter_op_num_threads").as_int();
        _nms_iou_threshold = static_cast<float>(get_parameter("nms_iou_threshold").as_double());
        _depth_estimation_mode = get_parameter("depth_estimation_mode").as_string();
        _confidence_threshold = static_cast<float>(get_parameter("confidence_threshold").as_double());
        _always_on.store(get_parameter("always_on").as_bool());  // set initial value of atomic bool
        _should_use_cuda = get_parameter("use_cuda").as_bool();
        _publish_annotated_image = get_parameter("publish_annotated_image").as_bool();
        _processing_frequency_hz = get_parameter("processing_frequency_hz").as_double();
        _model_path = std::string(std::getenv("HOME")) + get_parameter("model_path").as_string();

        if (_model_path.empty())
        {
            _model_path = ament_index_cpp::get_package_share_directory("perseus_vision") + "/models/cube_detector_yolob8s.onnx";
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

        _ort_session = std::make_unique<Ort::Session>(
            _ort_env, _model_path.c_str(), session_opts);

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

        // ── publishers ───────────────────────────────────────────────────────────────
        _pub_annotated = create_publisher<sensor_msgs::msg::Image>(
            "/perseus_vision/annotated_image", kQosDepth);
        _pub_colour = create_publisher<std_msgs::msg::String>(
            "/perseus_vision/detected_colour", kQosDepth);

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
        _always_on.store(get_parameter("always_on").as_bool());
        if (!_always_on.load())
        {
            return;  // skip processing if always_on is false and there are no subscribers
        }

        _processing_frequency_hz = get_parameter("processing_frequency_hz").as_double();
        if (_processing_frequency_hz > 0.0)
        {
            const int64_t now_ns = this->now().nanoseconds();
            const int64_t min_period_ns = static_cast<int64_t>(1e9 / _processing_frequency_hz);
            if (_last_inference_time_ns != 0 && (now_ns - _last_inference_time_ns) < min_period_ns)
            {
                return;
            }
            _last_inference_time_ns = now_ns;
        }

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

        // ── preprocess → infer ───────────────────────────────────────────────────
        auto blob = preprocess(cv_ptr->image);

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

        // output shape: [1, 8, 8400]
        const float* out_data = output_tensors[0].GetTensorData<float>();
        const auto out_shape = output_tensors[0].GetTensorTypeAndShapeInfo().GetShape();
        const size_t num_boxes = static_cast<size_t>(out_shape[2]);

        auto detections = postprocess(out_data, num_boxes);
        if (_publish_annotated_image)
        {
            publish_annotated_image(cv_ptr->image, detections, msg->header);
        }
        //   publish_detections(detections, msg->header);
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

        _pub_annotated->publish(
            *cv_bridge::CvImage(header, "bgr8", annotated).toImageMsg());
    }

}  // namespace perseus_vision
