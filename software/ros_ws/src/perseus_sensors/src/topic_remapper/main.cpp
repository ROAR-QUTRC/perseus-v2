/**
 * @file main.cpp
 * @brief Entry point for the topic_remapper node
 */

#include "rclcpp/rclcpp.hpp"
#include "topic_remapper/topic_remapper.hpp"

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  rclcpp::spin(std::make_shared<TopicRemapper>(options));

  rclcpp::shutdown();
  return 0;
}
