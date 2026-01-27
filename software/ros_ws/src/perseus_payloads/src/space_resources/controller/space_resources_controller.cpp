#include "rclcpp/rclcpp.hpp"
#include "space_resources/controller/space_resources_controller.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SpaceResourcesController>());
  rclcpp::shutdown();
  return 0;
}
