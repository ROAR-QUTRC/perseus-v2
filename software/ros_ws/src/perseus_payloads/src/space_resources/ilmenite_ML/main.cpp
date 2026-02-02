#include "space_resources/ilmenite_ML/main.hpp"

#include <memory>

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<IlmeniteML>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
