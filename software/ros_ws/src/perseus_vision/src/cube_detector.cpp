#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <onnxruntime_cxx_api.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include <cmath>

class CubeDetector : public rclcpp::Node {
public:
    CubeDetector()
    : Node("cube_detector")
    {
        RCLCPP_INFO(this->get_logger(), "Cube Detector Node Started");

        // Publisher for annotated image
        pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/rgbd_camera/image/cube_detection", 10);

        // Subscriber for raw camera image
        sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/rgbd_camera/image_raw", 10,
            std::bind(&CubeDetector::image_callback, this, std::placeholders::_1)
        );

        // Initialize ONNX Runtime session
        env_ = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "CubeDetector");
        Ort::SessionOptions session_options;
        session_options.SetIntraOpNumThreads(1);
        session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);

        std::string model_path = ament_index_cpp::get_package_share_directory("perseus_vision") 
                                 + "/yolo_model/best.onnx";

        RCLCPP_INFO(this->get_logger(), "Loading model from: %s", model_path.c_str());

        // Check if model file exists
        std::ifstream model_file(model_path);
        if (!model_file.good()) {
            RCLCPP_ERROR(this->get_logger(), "Model file not found: %s", model_path.c_str());
            RCLCPP_WARN(this->get_logger(), "Running in pass-through mode without detection");
            model_loaded_ = false;
        } else {
            try {
                session_ = std::make_unique<Ort::Session>(*env_, model_path.c_str(), session_options);
                allocator_ = std::make_unique<Ort::AllocatorWithDefaultOptions>();
                model_loaded_ = true;
                RCLCPP_INFO(this->get_logger(), "Model loaded successfully");
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Failed to load model: %s", e.what());
                RCLCPP_WARN(this->get_logger(), "Running in pass-through mode without detection");
                model_loaded_ = false;
            }
        }
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        // Convert ROS image to OpenCV
        cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;

        if (!model_loaded_) {
            auto out_msg = cv_bridge::CvImage(msg->header, "bgr8", frame).toImageMsg();
            pub_->publish(*out_msg);
            return;
        }

        // Preprocess
        cv::Mat input_blob;
        cv::resize(frame, input_blob, cv::Size(640, 640));
        input_blob.convertTo(input_blob, CV_32F, 1.0 / 255.0);

        std::vector<int64_t> input_shape = {1, 3, 640, 640};
        std::vector<float> input_tensor_values(input_blob.total() * input_blob.channels());
        std::memcpy(input_tensor_values.data(), input_blob.data, input_blob.total() * input_blob.elemSize());

        Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
            allocator_->GetInfo(),
            input_tensor_values.data(),
            input_tensor_values.size(),
            input_shape.data(),
            input_shape.size()
        );

        const char* input_names[] = {"images"};
        const char* output_names[] = {"output0"};

        auto output_tensors = session_->Run(
            Ort::RunOptions{nullptr},
            input_names,
            &input_tensor,
            1,
            output_names,
            1
        );

        // Parse output
        auto tensor_info = output_tensors[0].GetTensorTypeAndShapeInfo();
        auto shape = tensor_info.GetShape();

        if (shape.size() == 3 && shape[0] == 1 && shape[1] == 8 && shape[2] == 8400) {
            int num_features = shape[1];   // 8
            int num_detections = shape[2]; // 8400
            float* output_data = output_tensors[0].GetTensorMutableData<float>();

            float conf_threshold = 0.4;
            int detections_found = 0;
            int max_detections = 50; // Limit output to avoid spam

            // Process each detection
            for (int i = 0; i < num_detections && detections_found < max_detections; i++) {
                // Extract coordinates (first 4 features)
                float x = output_data[0 * num_detections + i];  // x center
                float y = output_data[1 * num_detections + i];  // y center
                float w = output_data[2 * num_detections + i];  // width
                float h = output_data[3 * num_detections + i];  // height

                // Extract class probabilities (features 4-7, not 5-8)
                float max_score = -1.0f;
                int best_class = -1;
                for (int cls = 4; cls < num_features; cls++) {
                    float score = output_data[cls * num_detections + i];
                    if (score > max_score) {
                        max_score = score;
                        best_class = cls - 4;  // Classes 0, 1, 2, 3
                    }
                }

                // Apply confidence threshold
                if (max_score > conf_threshold) {
                    detections_found++;
                    
                    // Convert from center/width/height to corner coordinates and scale to image size
                    int x1 = static_cast<int>((x - w/2) * frame.cols / 640.0f);
                    int y1 = static_cast<int>((y - h/2) * frame.rows / 640.0f);
                    int x2 = static_cast<int>((x + w/2) * frame.cols / 640.0f);
                    int y2 = static_cast<int>((y + h/2) * frame.rows / 640.0f);

                    // Clamp coordinates to image bounds
                    x1 = std::max(0, std::min(frame.cols-1, x1));
                    y1 = std::max(0, std::min(frame.rows-1, y1));
                    x2 = std::max(0, std::min(frame.cols-1, x2));
                    y2 = std::max(0, std::min(frame.rows-1, y2));

                    // Define colors for each class (BGR format for OpenCV)
                    cv::Scalar color;
                    switch (best_class) {
                        case 0: color = cv::Scalar(255, 0, 0);   break; // Blue
                        case 1: color = cv::Scalar(0, 255, 0);   break; // Green
                        case 2: color = cv::Scalar(0, 0, 255);   break; // Red
                        case 3: color = cv::Scalar(255, 255, 255); break; // White
                        default: color = cv::Scalar(0, 255, 255); break; // Yellow (unknown)
                    }

                    // Draw bounding box and label
                    cv::rectangle(frame, cv::Point(x1, y1), cv::Point(x2, y2), color, 2);
                    cv::putText(frame, "Class " + std::to_string(best_class) + " " +
                                std::to_string(max_score).substr(0,4),
                                cv::Point(x1, y1 - 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 2);

                    // Log detection (matching your Python output format)
                    RCLCPP_INFO(this->get_logger(),
                                "Class %d | Score: %.2f | Box: [%d, %d, %d, %d] | Color: (%d,%d,%d)",
                                best_class, max_score, x1, y1, x2, y2,
                                (int)color[0], (int)color[1], (int)color[2]);
                }
            }
            
            if (detections_found > 0) {
                RCLCPP_INFO(this->get_logger(), "Found %d detections", detections_found);
            }
            
        } else {
            RCLCPP_WARN(this->get_logger(), "Unexpected tensor shape - expected [1,8,8400], got [%ld,%ld,%ld]", 
                       shape[0], shape[1], shape[2]);
        }

        auto out_msg = cv_bridge::CvImage(msg->header, "bgr8", frame).toImageMsg();
        pub_->publish(*out_msg);
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    std::unique_ptr<Ort::Env> env_;
    std::unique_ptr<Ort::Session> session_;
    std::unique_ptr<Ort::AllocatorWithDefaultOptions> allocator_;
    bool model_loaded_ = false;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CubeDetector>());
    rclcpp::shutdown();
    return 0;
}
