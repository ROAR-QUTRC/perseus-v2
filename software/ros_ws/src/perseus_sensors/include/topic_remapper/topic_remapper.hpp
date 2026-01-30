#pragma once

#include <memory>
#include <string>
#include <functional>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp/message_info.hpp"
#include "rclcpp/generic_subscription.hpp"
#include "rclcpp/generic_publisher.hpp"

/**
 * @file topic_remapper.hpp
 * @brief Dynamic topic remapper node
 *
 * This node dynamically detects the message type of a topic and republishes it
 * at a lower frequency. It uses ROS 2's generic subscription and publisher capabilities
 * to handle any message type without compile-time knowledge of the specific types.
 */

class TopicRemapper : public rclcpp::Node
{
public:
  /**
   * @brief Constructor
   * @param options ROS node options
   */
  explicit TopicRemapper(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  /**
   * @brief Destructor
   */
  ~TopicRemapper();

private:
  /**
   * @brief Detects and subscribes to the input topic
   *
   * This method is called to set up the subscription after the topic name is known.
   * It automatically detects the message type and creates an appropriate subscription.
   */
  void detect_and_subscribe();

  /**
   * @brief Generic message callback that handles any message type
   *
   * @param msg The serialized message
   */
  void generic_message_callback(std::shared_ptr<rclcpp::SerializedMessage> msg);

  // ROS parameters
  std::string input_topic_name_;
  std::string output_topic_name_;
  double reduction_frequency_;  // Target output frequency in Hz
  std::string message_type_;    // Detected message type

  // ROS components
  std::shared_ptr<rclcpp::GenericSubscription> subscription_;
  std::shared_ptr<rclcpp::GenericPublisher> publisher_;

  // Timing for frequency reduction
  std::chrono::steady_clock::time_point last_publish_time_;
  std::chrono::milliseconds publish_interval_;

  // Flag to track if we've detected and set up the publisher
  bool is_setup_complete_;
};
