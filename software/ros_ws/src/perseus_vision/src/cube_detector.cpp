#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <onnxruntime_cxx_api.h>

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

        session_ = std::make_unique<Ort::Session>(*env_, "/home/username/perseus_vision/yolo_model/best.onnx", session_options);

        allocator_ = std::make_unique<Ort::AllocatorWithDefaultOptions>();
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        // Convert ROS image to OpenCV
        cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;

        // Preprocess: resize, normalize, etc. (example for YOLOv8 640x640)
        cv::Mat input_blob;
        cv::resize(frame, input_blob, cv::Size(640, 640));
        input_blob.convertTo(input_blob, CV_32F, 1.0/255.0);

        // TODO: Run ONNX Runtime inference
        // - Prepare input tensor
        // - Run session_->Run(...)
        // - Parse output: boxes, scores, class_ids

        // Dummy box example (replace with real inference)
        cv::rectangle(frame, cv::Point(100, 100), cv::Point(200, 200), cv::Scalar(0,0,255), 2);
        cv::putText(frame, "red_cube", cv::Point(100,90), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0,0,255), 2);

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
