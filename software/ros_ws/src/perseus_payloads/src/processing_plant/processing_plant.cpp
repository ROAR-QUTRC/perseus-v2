#include "processing_plant/processing_plant.hpp"

ProcessingPlant::ProcessingPlant(const rclcpp::NodeOptions& options)
    : Node("processing_plant", options)
{
    RCLCPP_INFO(this->get_logger(), "Processing plant node initialised");
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    try
    {
        auto node = std::make_shared<ProcessingPlant>();
        RCLCPP_INFO(rclcpp::get_logger("main"), "Starting bucket driver node");
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