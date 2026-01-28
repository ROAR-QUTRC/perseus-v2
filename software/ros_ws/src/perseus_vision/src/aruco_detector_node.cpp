#include "perseus_vision/aruco_detector.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<perseus_vision::ArucoDetector>());
    rclcpp::shutdown();
    return 0;
}
