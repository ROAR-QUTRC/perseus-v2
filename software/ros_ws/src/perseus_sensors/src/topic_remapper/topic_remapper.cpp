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
    : Node("topic_remapper", options),
      _detection_attempts(0),
      _is_setup_complete(false)
{
    RCLCPP_INFO(this->get_logger(), "Initializing TopicRemapper node...");

    // Declare parameters
    this->declare_parameter("input_topic", "");
    this->declare_parameter("output_topic", "remapped");
    this->declare_parameter("reduction_frequency", 10.0);  // Hz, default 10 Hz

    // Get parameters
    _input_topic_name = this->get_parameter("input_topic").as_string();
    _output_topic_name = this->get_parameter("output_topic").as_string();
    _reduction_frequency = this->get_parameter("reduction_frequency").as_double();

    // Validate parameters
    if (_input_topic_name.empty())
    {
        RCLCPP_ERROR(this->get_logger(), "input_topic parameter is required and cannot be empty");
        throw std::invalid_argument("input_topic parameter is required");
    }

    if (_reduction_frequency <= 0.0)
    {
        RCLCPP_ERROR(this->get_logger(), "reduction_frequency must be positive, got: %f", _reduction_frequency);
        throw std::invalid_argument("reduction_frequency must be positive");
    }

    // Calculate publish interval in milliseconds
    _publish_interval = std::chrono::milliseconds(static_cast<int64_t>(1000.0 / _reduction_frequency));

    RCLCPP_INFO(this->get_logger(), "Parameters loaded:");
    RCLCPP_INFO(this->get_logger(), "  Input topic: %s", _input_topic_name.c_str());
    RCLCPP_INFO(this->get_logger(), "  Output topic: %s", _output_topic_name.c_str());
    RCLCPP_INFO(this->get_logger(), "  Reduction frequency: %.2f Hz (interval: %ld ms)", _reduction_frequency,
                _publish_interval.count());

    // Initialize timing
    _last_publish_time = std::chrono::steady_clock::now();

    // Start detection with a timer that retries periodically
    _detection_timer = this->create_wall_timer(100ms, [this]()
                                               { this->detect_and_subscribe(); });
}

TopicRemapper::~TopicRemapper() { RCLCPP_INFO(this->get_logger(), "TopicRemapper node shutting down"); }

void TopicRemapper::detect_and_subscribe()
{
    // Only run once setup is complete
    if (_is_setup_complete)
    {
        return;
    }

    _detection_attempts++;

    if (_detection_attempts == 1)
    {
        RCLCPP_INFO(this->get_logger(), "Attempting to detect message type on topic: %s", _input_topic_name.c_str());
    }

    // Get topic names and types from the node graph
    auto topic_names_and_types = this->get_topic_names_and_types();

    bool topic_found = false;
    for (const auto& [topic_name, types] : topic_names_and_types)
    {
        if (topic_name == _input_topic_name && !types.empty())
        {
            _message_type = types[0];
            topic_found = true;
            break;
        }
    }

    if (!topic_found)
    {
        if (_detection_attempts == 1)
        {
            RCLCPP_INFO(this->get_logger(), "Topic '%s' not found yet. Waiting for publisher...", _input_topic_name.c_str());
        }
        else if (_detection_attempts % 10 == 0)
        {
            RCLCPP_DEBUG(this->get_logger(), "Still waiting for topic '%s'... (attempt %d/%d)", _input_topic_name.c_str(),
                         _detection_attempts, MAX_DETECTION_ATTEMPTS);
        }

        if (_detection_attempts >= MAX_DETECTION_ATTEMPTS)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to detect message type after %d attempts. Topic '%s' may not exist.",
                         MAX_DETECTION_ATTEMPTS, _input_topic_name.c_str());
            _detection_timer->cancel();
            return;
        }

        return;  // Wait for next attempt
    }

    RCLCPP_INFO(this->get_logger(), "Detected message type for topic '%s': %s", _input_topic_name.c_str(),
                _message_type.c_str());

    // Cancel the detection timer since we found the topic
    _detection_timer->cancel();

    try
    {
        // Create generic publisher
        _publisher = this->create_generic_publisher(_output_topic_name, _message_type, 10);
        RCLCPP_INFO(this->get_logger(), "Created generic publisher on topic: %s", _output_topic_name.c_str());

        // Create generic subscription
        _subscription = this->create_generic_subscription(
            _input_topic_name, _message_type, 10,
            std::bind(&TopicRemapper::generic_message_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Created generic subscription on topic: %s", _input_topic_name.c_str());
        RCLCPP_INFO(this->get_logger(), "TopicRemapper is now active and will republish at %.2f Hz", _reduction_frequency);

        _is_setup_complete = true;
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to create subscription/publisher: %s", e.what());
        _detection_timer->cancel();
    }
}

void TopicRemapper::generic_message_callback(std::shared_ptr<rclcpp::SerializedMessage> msg)
{
    if (!_is_setup_complete)
    {
        return;
    }

    // Check if enough time has passed since the last publish
    auto now = std::chrono::steady_clock::now();
    auto time_since_last_publish = std::chrono::duration_cast<std::chrono::milliseconds>(now - _last_publish_time);

    if (time_since_last_publish >= _publish_interval)
    {
        // Publish the message
        _publisher->publish(*msg);
        _last_publish_time = now;

        RCLCPP_DEBUG(this->get_logger(), "Published message from '%s' to '%s'", _input_topic_name.c_str(),
                     _output_topic_name.c_str());
    }
    else
    {
        RCLCPP_DEBUG(this->get_logger(), "Skipping message: only %ld ms since last publish (need %ld ms)",
                     time_since_last_publish.count(), _publish_interval.count());
    }
}
