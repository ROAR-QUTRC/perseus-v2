#include <rclcpp/rclcpp.hpp>

class MapperNode : public rclcpp::Node
{
public:
  MapperNode() : Node("mapper")
  {
    RCLCPP_INFO(this->get_logger(), "Mapper node has been started");
  }
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MapperNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
