#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

class ArucoDetector : public rclcpp::Node
{
public:
    ArucoDetector()
        : Node("aruco_detector")
    {
        // Camera parameters (replace with calibrated values)
        camera_matrix_ = (cv::Mat_<double>(3, 3) << 600, 0, 320,
                                                    0, 600, 240,
                                                    0, 0, 1);
        dist_coeffs_ = cv::Mat::zeros(5, 1, CV_64F);

        // Create ArUco dictionary and detector
        dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100);  // Load a predefined dictionary
        detector_ = cv::aruco::ArucoDetector(dictionary_); // Create the detector with the dictionary

        // Subscriber
        sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10,
            std::bind(&ArucoDetector::imageCallback, this, std::placeholders::_1)); // Subscribe to raw camera images

        // Publisher
        pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/processed_aruco/image_raw", 10); // Publish processed images with aruco markers
 
        RCLCPP_INFO(this->get_logger(), "Perseus' ArucoDetector node started.");
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // Convert ROS Image to OpenCV Mat
        cv::Mat frame;
        try
        {
            frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        // Detect markers
        std::vector<int> ids; // list of detected marker IDs
        std::vector<std::vector<cv::Point2f>> corners; 
        detector_.detectMarkers(frame, corners, ids); // detect markers

        if (!ids.empty())
        {
            // Estimate pose for each marker
            std::vector<cv::Vec3d> rvecs, tvecs;
            cv::aruco::estimatePoseSingleMarkers(corners, 0.05, camera_matrix_, dist_coeffs_, rvecs, tvecs);

            for (size_t i = 0; i < ids.size(); ++i)
            {
                cv::aruco::drawDetectedMarkers(frame, corners, ids);
                cv::drawFrameAxes(frame, camera_matrix_, dist_coeffs_, rvecs[i], tvecs[i], 0.03);

                // Center coordinates
                cv::Point2f center(0, 0);
                for (const auto &pt : corners[i])
                    center += pt;
                center *= (1.0f / 4.0f);

                RCLCPP_INFO(this->get_logger(), "Marker ID: %d, Center: (%.1f, %.1f)",
                            ids[i], center.x, center.y);

                cv::putText(frame, "(" + std::to_string((int)center.x) + "," +
                                     std::to_string((int)center.y) + ")",
                            corners[i][0] + cv::Point2f(0, -30),
                            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 2);
            }
        }

        // Publish processed frame
        auto processed_msg = cv_bridge::CvImage(msg->header, "bgr8", frame).toImageMsg();
        pub_->publish(*processed_msg);
    }

    // ROS interfaces
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_; // Subscriber for raw camera images
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_; // Publisher for processed images

    // ArUco
    cv::aruco::Dictionary dictionary_;
    cv::aruco::ArucoDetector detector_;

    // Camera intrinsics
    cv::Mat camera_matrix_, dist_coeffs_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArucoDetector>());
    rclcpp::shutdown();
    return 0;
}
