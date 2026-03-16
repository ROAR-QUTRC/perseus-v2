#include "perseus_vision/cube_detector.hpp"

#include <algorithm>
#include <string>
#include <vector>

namespace perseus_vision
{

// ── constructor ───────────────────────────────────────────────────────────────
CubeDetector::CubeDetector()
: Node("cube_detector"),
  ort_env_(ORT_LOGGING_LEVEL_WARNING, "cube_detector")
{
  // ── parameters ──────────────────────────────────────────────────────────────
    declare_parameter("model_path", std::string(std::getenv("HOME")) + 
    "/perseus-v2/software/ros_ws/src/perseus_vision/models/cube_detector.onnx");
  declare_parameter("confidence_threshold", 0.5);
  declare_parameter("camera_topic",         "/camera/camera/color/image_raw");
  declare_parameter("camera_info_topic",    "/camera/camera/color/camera_info");

        const auto camera_topic = get_parameter("camera_topic").as_string();
        const auto camera_info_topic = get_parameter("camera_info_topic").as_string();
        confidence_threshold_ = static_cast<float>(
            get_parameter("confidence_threshold").as_double());

        model_path_ = get_parameter("model_path").as_string();
        //   if (model_path_.empty()) {
        //     model_path_ = ament_index_cpp::get_package_share_directory("perseus_vision")
        //                 + "/models/best.onnx";
        //   }

        // ── load ONNX model ──────────────────────────────────────────────────────────
        RCLCPP_INFO(get_logger(), "Loading model: %s", model_path_.c_str());

        Ort::SessionOptions session_opts;
        session_opts.SetIntraOpNumThreads(1);
        session_opts.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);

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

        ort_session_ = std::make_unique<Ort::Session>(
            ort_env_, model_path_.c_str(), session_opts);

        input_name_ = ort_session_->GetInputNameAllocated(0, ort_allocator_).get();
        output_name_ = ort_session_->GetOutputNameAllocated(0, ort_allocator_).get();

        RCLCPP_INFO(get_logger(), "Model loaded — input: %s  output: %s",
                    input_name_.c_str(), output_name_.c_str());

        // ── subscribers ──────────────────────────────────────────────────────────────
        sub_image_ = create_subscription<sensor_msgs::msg::Image>(
            camera_topic, kQosDepth,
            std::bind(&CubeDetector::image_callback, this, std::placeholders::_1));

        sub_camera_info_ = create_subscription<sensor_msgs::msg::CameraInfo>(
            camera_info_topic, kQosDepth,
            std::bind(&CubeDetector::camera_info_callback, this, std::placeholders::_1));

        // ── publishers ───────────────────────────────────────────────────────────────
        pub_annotated_ = create_publisher<sensor_msgs::msg::Image>(
            "/perseus_vision/annotated_image", kQosDepth);

        //   pub_detections_ = create_publisher<vision_msgs::msg::Detection2DArray>(
        //     "/perseus_vision/detections", kQosDepth);

        pub_colour_ = create_publisher<std_msgs::msg::String>(
            "/perseus_vision/detected_colour", kQosDepth);

        RCLCPP_INFO(get_logger(), "CubeDetectorNode ready — subscribed to %s", camera_topic.c_str());
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

        // ── preprocess → infer ───────────────────────────────────────────────────
        auto blob = preprocess(cv_ptr->image);

        std::array<int64_t, 4> input_shape = {1, 3, 640, 640};
        Ort::MemoryInfo mem_info =
            Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);

        Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
            mem_info, blob.data(), blob.size(),
            input_shape.data(), input_shape.size());

        const char* input_names[] = {input_name_.c_str()};
        const char* output_names[] = {output_name_.c_str()};

        auto output_tensors = ort_session_->Run(
            Ort::RunOptions{nullptr},
            input_names, &input_tensor, 1,
            output_names, 1);

        // output shape: [1, 8, 8400]
        const float* out_data = output_tensors[0].GetTensorData<float>();
        const auto out_shape = output_tensors[0].GetTensorTypeAndShapeInfo().GetShape();
        const size_t num_boxes = static_cast<size_t>(out_shape[2]);

        auto detections = postprocess(out_data, num_boxes);

        publish_annotated_image(cv_ptr->image, detections, msg->header);
        //   publish_detections(detections, msg->header);
    }

    // ── camera info callback (kept for future depth use) ─────────────────────────
    void CubeDetector::camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(camera_matrix_mutex_);

        camera_matrix_ = cv::Mat(3, 3, CV_64F);
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                camera_matrix_.at<double>(r, c) = msg->k[r * 3 + c];

        dist_coeffs_ = cv::Mat(static_cast<int>(msg->d.size()), 1, CV_64F);
        for (size_t i = 0; i < msg->d.size(); ++i)
            dist_coeffs_.at<double>(static_cast<int>(i), 0) = msg->d[i];
    }

    // ── preprocess ────────────────────────────────────────────────────────────────
    std::vector<float> CubeDetector::preprocess(const cv::Mat& bgr_image)
    {
        orig_h_ = bgr_image.rows;
        orig_w_ = bgr_image.cols;

        cv::Mat resized, rgb;
        cv::resize(bgr_image, resized, cv::Size(640, 640));
        cv::cvtColor(resized, rgb, cv::COLOR_BGR2RGB);
        rgb.convertTo(rgb, CV_32FC3, 1.0 / 255.0);

        std::vector<cv::Mat> channels(3);
        cv::split(rgb, channels);

        std::vector<float> blob;
        blob.reserve(3 * 640 * 640);
        for (auto& ch : channels)
        {
            blob.insert(blob.end(),
                        reinterpret_cast<float*>(ch.data),
                        reinterpret_cast<float*>(ch.data) + 640 * 640);
        }
        return blob;
    }

    // ── postprocess ───────────────────────────────────────────────────────────────
    std::vector<Detection> CubeDetector::postprocess(const float* data, size_t num_boxes)
    {
        const float scale_x = static_cast<float>(orig_w_) / 640.0f;
        const float scale_y = static_cast<float>(orig_h_) / 640.0f;
        const int num_classes = static_cast<int>(CLASS_NAMES.size());

        std::vector<Detection> detections;

        for (size_t i = 0; i < num_boxes; ++i)
        {
            // data layout: [field][box] → field f, box i = data[f * num_boxes + i]
            const float xc = data[0 * num_boxes + i];
            const float yc = data[1 * num_boxes + i];
            const float w = data[2 * num_boxes + i];
            const float h = data[3 * num_boxes + i];

            int best_class = 0;
            float best_score = 0.0f;
            for (int c = 0; c < num_classes; ++c)
            {
                const float score = data[(4 + c) * num_boxes + i];
                if (score > best_score)
                {
                    best_score = score;
                    best_class = c;
                }
            }

            if (best_score < confidence_threshold_)
                continue;

            const int x1 = std::clamp(static_cast<int>((xc - w / 2.0f) * scale_x), 0, orig_w_);
            const int y1 = std::clamp(static_cast<int>((yc - h / 2.0f) * scale_y), 0, orig_h_);
            const int x2 = std::clamp(static_cast<int>((xc + w / 2.0f) * scale_x), 0, orig_w_);
            const int y2 = std::clamp(static_cast<int>((yc + h / 2.0f) * scale_y), 0, orig_h_);

            detections.push_back({best_class, best_score, cv::Rect(x1, y1, x2 - x1, y2 - y1)});
        }

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

        pub_annotated_->publish(
            *cv_bridge::CvImage(header, "bgr8", annotated).toImageMsg());
    }

    // ── publish detections ────────────────────────────────────────────────────────
    // void CubeDetector::publish_detections(
    //   const std::vector<Detection> & detections,
    //   const std_msgs::msg::Header & header)
    // {
    //   vision_msgs::msg::Detection2DArray det_array;
    //   det_array.header = header;

    //   std::string best_colour;
    //   float       best_conf = 0.0f;

    //   for (const auto & d : detections) {
    //     vision_msgs::msg::Detection2D det;
    //     det.header = header;

    //     // hypothesis: class + confidence
    //     vision_msgs::msg::ObjectHypothesisWithPose hyp;
    //     hyp.hypothesis.class_id = CLASS_NAMES[d.class_id];  // "blue" not "0"
    //     hyp.hypothesis.score    = static_cast<double>(d.confidence);
    //     det.results.push_back(hyp);

    //     // bounding box center + size in pixel coordinates
    //     det.bbox.center.position.x = static_cast<double>(d.bbox.x + d.bbox.width  / 2);
    //     det.bbox.center.position.y = static_cast<double>(d.bbox.y + d.bbox.height / 2);
    //     det.bbox.size_x            = static_cast<double>(d.bbox.width);
    //     det.bbox.size_y            = static_cast<double>(d.bbox.height);

    //     det_array.detections.push_back(det);

    //     if (d.confidence > best_conf) {
    //       best_conf   = d.confidence;
    //       best_colour = CLASS_NAMES[d.class_id];
    //     }
    //   }

    //   pub_detections_->publish(det_array);

    //   // publish highest confidence colour separately
    //   if (!best_colour.empty()) {
    //     std_msgs::msg::String colour_msg;
    //     colour_msg.data = best_colour;
    //     pub_colour_->publish(colour_msg);
    //   }
    // }

}  // namespace perseus_vision