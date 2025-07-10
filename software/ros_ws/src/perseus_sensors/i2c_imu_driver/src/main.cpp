#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "i2c_imu_driver/i2c_imu_node.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    try {
        auto node = std::make_shared<i2c_imu_driver::I2cImuNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("i2c_imu_main"), "Failed to start I2C IMU node: %s", e.what());
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}