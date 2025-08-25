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
        pub_ = this->create_publisher<sensor_msgs::msg::Image>("/rgbd_camera/image/cube_detection", 10);

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

        std::string model_path = ament_index_cpp::get_package_share_directory("perseus_vision") + "/yolo_model/best.onnx";
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

        // If model is not loaded, just pass through the image
        if (!model_loaded_) {
            auto out_msg = cv_bridge::CvImage(msg->header, "bgr8", frame).toImageMsg();
            pub_->publish(*out_msg);
            return;
        }

        // Preprocess: resize, normalize, etc. (example for YOLOv8 640x640)
        cv::Mat input_blob;
        cv::resize(frame, input_blob, cv::Size(640, 640));
        input_blob.convertTo(input_blob, CV_32F, 1.0 / 255.0);
        
        RCLCPP_INFO(this->get_logger(), "Original image size: %dx%d, Model input: 640x640", 
                   frame.cols, frame.rows);

        // Prepare input tensor
        std::vector<int64_t> input_shape = {1, 3, 640, 640}; // Batch size, channels, height, width
        std::vector<float> input_tensor_values(input_blob.total() * input_blob.channels());
        std::memcpy(input_tensor_values.data(), input_blob.data, input_blob.total() * input_blob.elemSize());

        Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
            allocator_->GetInfo(),
            input_tensor_values.data(),
            input_tensor_values.size(),
            input_shape.data(),
            input_shape.size()
        );

        // Run ONNX Runtime inference
        const char* input_names[] = {"images"};  // Replace with your model's input name
        const char* output_names[] = {"output0"}; // Replace with your model's output name
        auto output_tensors = session_->Run(
            Ort::RunOptions{nullptr},
            input_names,
            &input_tensor,
            1,
            output_names,
            1
        );

        // Parse output: boxes, scores, class_ids
        float* output_data = output_tensors[0].GetTensorMutableData<float>();
        size_t output_size = output_tensors[0].GetTensorTypeAndShapeInfo().GetElementCount();
        
        // Debug: Print output tensor shape and first few values
        auto tensor_info = output_tensors[0].GetTensorTypeAndShapeInfo();
        auto shape = tensor_info.GetShape();
        
        RCLCPP_INFO(this->get_logger(), "Output tensor dimensions: %zu", shape.size());
        for (size_t i = 0; i < shape.size(); i++) {
            RCLCPP_INFO(this->get_logger(), "  Dimension %zu: %ld", i, shape[i]);
        }
        RCLCPP_INFO(this->get_logger(), "Output size: %zu", output_size);
        
        // Print first 20 values to understand the format
        RCLCPP_INFO(this->get_logger(), "First 20 output values:");
        for (size_t i = 0; i < std::min(output_size, (size_t)20); i++) {
            RCLCPP_INFO(this->get_logger(), "  [%zu] = %.6f", i, output_data[i]);
        }

        // ACTUAL FORMAT DETECTED: [1, 8, 8400] - This is transposed YOLO format!
        // Standard YOLO is [1, 8400, 8] but this model outputs [1, 8, 8400]
        // Where 8 = [x, y, w, h, objectness, class_0, class_1, class_2, ...]
        
        if (shape.size() == 3 && shape[1] == 8 && shape[2] == 8400) {
            int num_features = shape[1];      // 8 features per detection
            int num_detections = shape[2];    // 8400 potential detections
            
            RCLCPP_INFO(this->get_logger(), "Transposed YOLO format: %d features, %d detections", 
                       num_features, num_detections);
            
            // Debug: Let's examine the first few detections in detail
            RCLCPP_INFO(this->get_logger(), "Examining first 10 detections in detail:");
            for (int i = 0; i < 10; i++) {
                float x_center = output_data[0 * num_detections + i];
                float y_center = output_data[1 * num_detections + i];
                float width = output_data[2 * num_detections + i];
                float height = output_data[3 * num_detections + i];
                float objectness = output_data[4 * num_detections + i];
                float class_0 = output_data[5 * num_detections + i];
                float class_1 = output_data[6 * num_detections + i];
                float class_2 = output_data[7 * num_detections + i];
                
                float sigmoid_objectness = 1.0f / (1.0f + expf(-objectness));
                float sigmoid_class_0 = 1.0f / (1.0f + expf(-class_0));
                float sigmoid_class_1 = 1.0f / (1.0f + expf(-class_1));
                float sigmoid_class_2 = 1.0f / (1.0f + expf(-class_2));
                
                RCLCPP_INFO(this->get_logger(), "  Det[%d]: x=%.3f, y=%.3f, w=%.3f, h=%.3f", 
                           i, x_center, y_center, width, height);
                RCLCPP_INFO(this->get_logger(), "          raw_obj=%.6f->sigmoid=%.6f", objectness, sigmoid_objectness);
                RCLCPP_INFO(this->get_logger(), "          raw_cls0=%.6f->sigmoid=%.6f", class_0, sigmoid_class_0);
                RCLCPP_INFO(this->get_logger(), "          raw_cls1=%.6f->sigmoid=%.6f", class_1, sigmoid_class_1);
                RCLCPP_INFO(this->get_logger(), "          raw_cls2=%.6f->sigmoid=%.6f", class_2, sigmoid_class_2);
            }
            
            // Process detections with lower threshold for testing
            int detections_found = 0;
            int max_to_show = 10;
            
            for (int i = 0; i < num_detections && detections_found < max_to_show; i++) {
                // In transposed format: feature_data[feature_index][detection_index]
                float x_center = output_data[0 * num_detections + i];   // Feature 0, detection i
                float y_center = output_data[1 * num_detections + i];   // Feature 1, detection i
                float width = output_data[2 * num_detections + i];      // Feature 2, detection i
                float height = output_data[3 * num_detections + i];     // Feature 3, detection i
                float objectness = output_data[4 * num_detections + i]; // Feature 4, detection i
                
                // Get class confidences (features 5, 6, 7 for classes 0, 1, 2, ...)
                float max_class_conf = -1000.0f;  // Start with very low value
                int best_class = -1;
                for (int cls = 5; cls < num_features; cls++) {
                    float class_conf = output_data[cls * num_detections + i];
                    if (class_conf > max_class_conf) {
                        max_class_conf = class_conf;
                        best_class = cls - 5;  // Class 0, 1, 2, ...
                    }
                }
                
                // Apply sigmoid to get proper probabilities (YOLO outputs logits)
                float sigmoid_objectness = 1.0f / (1.0f + expf(-objectness));
                float sigmoid_class_conf = 1.0f / (1.0f + expf(-max_class_conf));
                float final_confidence = sigmoid_objectness * sigmoid_class_conf;
                
                // Lower threshold for testing - let's see what we get
                if (final_confidence > 0.05) {  // Even lower threshold for debugging
                    detections_found++;
                    
                    RCLCPP_INFO(this->get_logger(), "Detection %d: center=[%.1f,%.1f], size=[%.1f,%.1f], obj=%.3f, cls_conf=%.3f, final_conf=%.3f, class=%d", 
                               i, x_center, y_center, width, height, sigmoid_objectness, sigmoid_class_conf, final_confidence, best_class);
                    
                    // Check if coordinates are already in pixel space or normalized
                    if (x_center > 1.0f || y_center > 1.0f || width > 1.0f || height > 1.0f) {
                        // Coordinates appear to be in pixel space already, use directly
                        float x1 = x_center - width / 2.0f;
                        float y1 = y_center - height / 2.0f;
                        float x2 = x_center + width / 2.0f;
                        float y2 = y_center + height / 2.0f;
                        
                        // Ensure coordinates are within image bounds
                        x1 = std::max(0.0f, std::min((float)frame.cols, x1));
                        y1 = std::max(0.0f, std::min((float)frame.rows, y1));
                        x2 = std::max(0.0f, std::min((float)frame.cols, x2));
                        y2 = std::max(0.0f, std::min((float)frame.rows, y2));
                        
                        cv::rectangle(frame, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(0, 255, 0), 2);
                        cv::putText(frame, "cube_" + std::to_string(best_class), cv::Point(x1, y1 - 10),
                                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
                        
                        RCLCPP_INFO(this->get_logger(), "CUBE DETECTED: class=%d, conf=%.3f, bbox=[%.1f,%.1f,%.1f,%.1f]", 
                                   best_class, final_confidence, x1, y1, x2, y2);
                    } else {
                        // Coordinates are normalized, scale to image size
                        float x1 = (x_center - width / 2.0f) * frame.cols;
                        float y1 = (y_center - height / 2.0f) * frame.rows;
                        float x2 = (x_center + width / 2.0f) * frame.cols;
                        float y2 = (y_center + height / 2.0f) * frame.rows;
                        
                        // Ensure coordinates are within image bounds
                        x1 = std::max(0.0f, std::min((float)frame.cols, x1));
                        y1 = std::max(0.0f, std::min((float)frame.rows, y1));
                        x2 = std::max(0.0f, std::min((float)frame.cols, x2));
                        y2 = std::max(0.0f, std::min((float)frame.rows, y2));
                        
                        cv::rectangle(frame, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(0, 255, 0), 2);
                        cv::putText(frame, "cube_" + std::to_string(best_class), cv::Point(x1, y1 - 10),
                                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
                        
                        RCLCPP_INFO(this->get_logger(), "CUBE DETECTED: class=%d, conf=%.3f, bbox=[%.1f,%.1f,%.1f,%.1f]", 
                                   best_class, final_confidence, x1, y1, x2, y2);
                    }
                }
            }
            
            RCLCPP_INFO(this->get_logger(), "Found %d high-confidence detections", detections_found);
            
        } else {
            RCLCPP_WARN(this->get_logger(), "Unexpected tensor shape - expected [1,8,8400], got [%ld,%ld,%ld]", 
                       shape[0], shape[1], shape[2]);
        }

        // Convert back to ROS Image and publish
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
