#include "space_resources/controller/space_resources_controller.hpp"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SpaceResourcesController>());
    rclcpp::shutdown();
    return 0;
}
