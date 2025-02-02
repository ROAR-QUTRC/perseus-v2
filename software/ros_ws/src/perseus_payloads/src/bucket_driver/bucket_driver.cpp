#include "bucket_driver/bucket_driver.hpp"

#include "rclcpp/rclcpp.hpp"

BucketDriver::BucketDriver(const rclcpp::NodeOptions& options)
    : Node("bucket_driver", options)
{
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    try
    {
        auto node = std::make_shared<BucketDriver>();
        RCLCPP_INFO(rclcpp::get_logger("main"), "Starting bucket driver node...");
        rclcpp::spin(node);
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Error running bucket driver: %s", e.what());
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
