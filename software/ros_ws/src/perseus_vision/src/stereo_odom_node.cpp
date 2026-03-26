#include "perseus_vision/stereo_odom.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<perseus_vision::StereoOdom>());
    rclcpp::shutdown();
    return 0;
}
