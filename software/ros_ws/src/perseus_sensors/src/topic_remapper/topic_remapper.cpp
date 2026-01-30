/**
 * @file topic_remapper.cpp
 * @brief Implementation of dynamic topic remapper
 *
 * This node dynamically detects the message type of a topic and republishes it
 * at a lower frequency.
 */

#include "topic_remapper/topic_remapper.hpp"

#include <chrono>
#include <memory>

using namespace std::chrono_literals;

TopicRemapper::TopicRemapper(const rclcpp::NodeOptions& options)
    : Node("topic_remapper", options), is_setup_complete_(false), detection_attempts_(0)
{
  RCLCPP_INFO(this->get_logger(), "Initializing TopicRemapper node...");

  // Declare parameters
  this->declare_parameter("input_topic", "");
  this->declare_parameter("output_topic", "remapped");
  this->declare_parameter("reduction_frequency", 10.0);  // Hz, default 10 Hz

  // Get parameters
  input_topic_name_ = this->get_parameter("input_topic").as_string();
  output_topic_name_ = this->get_parameter("output_topic").as_string();
  reduction_frequency_ = this->get_parameter("reduction_frequency").as_double();

  // Validate parameters
  if (input_topic_name_.empty())
  {
    RCLCPP_ERROR(this->get_logger(), "input_topic parameter is required and cannot be empty");
    throw std::invalid_argument("input_topic parameter is required");
  }

  if (reduction_frequency_ <= 0.0)
  {
    RCLCPP_ERROR(this->get_logger(), "reduction_frequency must be positive, got: %f", reduction_frequency_);
    throw std::invalid_argument("reduction_frequency must be positive");
  }

  // Calculate publish interval in milliseconds
  publish_interval_ = std::chrono::milliseconds(static_cast<int64_t>(1000.0 / reduction_frequency_));

  RCLCPP_INFO(this->get_logger(), "Parameters loaded:");
  RCLCPP_INFO(this->get_logger(), "  Input topic: %s", input_topic_name_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Output topic: %s", output_topic_name_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Reduction frequency: %.2f Hz (interval: %ld ms)", reduction_frequency_,
              publish_interval_.count());

  // Initialize timing
  last_publish_time_ = std::chrono::steady_clock::now();

  // Start detection with a timer that retries periodically
  detection_timer_ = this->create_wall_timer(100ms, [this]() { this->detect_and_subscribe(); });
}

TopicRemapper::~TopicRemapper() { RCLCPP_INFO(this->get_logger(), "TopicRemapper node shutting down"); }

void TopicRemapper::detect_and_subscribe()
{
  // Only run once setup is complete
  if (is_setup_complete_)
  {
    return;
  }

  detection_attempts_++;

  if (detection_attempts_ == 1)
  {
    RCLCPP_INFO(this->get_logger(), "Attempting to detect message type on topic: %s", input_topic_name_.c_str());
  }

  // Get topic names and types from the node graph
  auto topic_names_and_types = this->get_topic_names_and_types();

  bool topic_found = false;
  for (const auto& [topic_name, types] : topic_names_and_types)
  {
    if (topic_name == input_topic_name_ && !types.empty())
    {
      message_type_ = types[0];
      topic_found = true;
      break;
    }
  }

  if (!topic_found)
  {
    if (detection_attempts_ == 1)
    {
      RCLCPP_INFO(this->get_logger(), "Topic '%s' not found yet. Waiting for publisher...", input_topic_name_.c_str());
    }
    else if (detection_attempts_ % 10 == 0)
    {
      RCLCPP_DEBUG(this->get_logger(), "Still waiting for topic '%s'... (attempt %d/%d)", input_topic_name_.c_str(),
                   detection_attempts_, MAX_DETECTION_ATTEMPTS);
    }

    if (detection_attempts_ >= MAX_DETECTION_ATTEMPTS)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to detect message type after %d attempts. Topic '%s' may not exist.",
                   MAX_DETECTION_ATTEMPTS, input_topic_name_.c_str());
      detection_timer_->cancel();
      return;
    }

    return;  // Wait for next attempt
  }

  RCLCPP_INFO(this->get_logger(), "Detected message type for topic '%s': %s", input_topic_name_.c_str(),
              message_type_.c_str());

  // Cancel the detection timer since we found the topic
  detection_timer_->cancel();

  try
  {
    // Create generic publisher
    publisher_ = this->create_generic_publisher(output_topic_name_, message_type_, 10);
    RCLCPP_INFO(this->get_logger(), "Created generic publisher on topic: %s", output_topic_name_.c_str());

    // Create generic subscription
    subscription_ = this->create_generic_subscription(
        input_topic_name_, message_type_, 10,
        std::bind(&TopicRemapper::generic_message_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Created generic subscription on topic: %s", input_topic_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "TopicRemapper is now active and will republish at %.2f Hz", reduction_frequency_);

    is_setup_complete_ = true;
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to create subscription/publisher: %s", e.what());
    detection_timer_->cancel();
  }
}

void TopicRemapper::generic_message_callback(std::shared_ptr<rclcpp::SerializedMessage> msg)
{
  if (!is_setup_complete_)
  {
    return;
  }

  // Check if enough time has passed since the last publish
  auto now = std::chrono::steady_clock::now();
  auto time_since_last_publish = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_publish_time_);

  if (time_since_last_publish >= publish_interval_)
  {
    // Publish the message
    publisher_->publish(*msg);
    last_publish_time_ = now;

    RCLCPP_DEBUG(this->get_logger(), "Published message from '%s' to '%s'", input_topic_name_.c_str(),
                 output_topic_name_.c_str());
  }
  else
  {
    RCLCPP_DEBUG(this->get_logger(), "Skipping message: only %ld ms since last publish (need %ld ms)",
                 time_since_last_publish.count(), publish_interval_.count());
  }
}
