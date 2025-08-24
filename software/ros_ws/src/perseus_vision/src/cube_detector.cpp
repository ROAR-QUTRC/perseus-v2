#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <onnxruntime_cxx_api.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

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

        session_ = std::make_unique<Ort::Session>(*env_, model_path.c_str(), session_options);

        allocator_ = std::make_unique<Ort::AllocatorWithDefaultOptions>();
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        // Convert ROS image to OpenCV
        cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;

        // Preprocess: resize, normalize, etc. (example for YOLOv8 640x640)
        cv::Mat input_blob;
        cv::resize(frame, input_blob, cv::Size(640, 640));
        input_blob.convertTo(input_blob, CV_32F, 1.0 / 255.0);

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
        const char* output_names[] = {"output"}; // Replace with your model's output name
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

        // Example: Iterate over the output (replace with your model's output format)
        for (size_t i = 0; i < output_size; i += 6) { // Assuming YOLO format: [x1, y1, x2, y2, score, class_id]
            float x1 = output_data[i];
            float y1 = output_data[i + 1];
            float x2 = output_data[i + 2];
            float y2 = output_data[i + 3];
            float score = output_data[i + 4];
            int class_id = static_cast<int>(output_data[i + 5]);

            if (score > 0.5) { // Confidence threshold
                cv::rectangle(frame, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(0, 255, 0), 2);
                cv::putText(frame, "class_" + std::to_string(class_id), cv::Point(x1, y1 - 10),
                            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
            }
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
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CubeDetector>());
    rclcpp::shutdown();
    return 0;
}
