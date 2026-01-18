#include <chrono>
#include <mutex>
#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/action_node.h"

#include "perseus_vision/srv/detect_objects.hpp"

using namespace std::chrono_literals;

namespace perseus_bt_nodes
{
class DetectAruco : public BT::StatefulActionNode
{
public:
  DetectAruco(const std::string& name, const BT::NodeConfiguration& config)
  : BT::StatefulActionNode(name, config)
  {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("service_name", "/detect_objects"),
      BT::InputPort<double>("timeout_sec", 0.2, "Service call timeout (s)"),
      BT::InputPort<double>("max_age_sec", 0.5, "Max allowed age of detections (s)"),
      BT::InputPort<int>("min_count", 1, "Minimum number of detections required for SUCCESS"),
      BT::OutputPort<std::vector<int>>("ids"),
      BT::OutputPort<std::vector<geometry_msgs::msg::Pose>>("poses"),
      BT::OutputPort<std::string>("frame_id")
    };
  }

  BT::NodeStatus onStart() override
  {
    // Create a ROS node for this BT plugin (once)
    if (!node_) {
      node_ = rclcpp::Node::make_shared("bt_detect_aruco_client");
    }

    // Create/refresh the client if service name changed
    std::string service_name;
    getInput("service_name", service_name);

    if (!client_ || service_name_ != service_name) {
      service_name_ = service_name;
      client_ = node_->create_client<perseus_vision::srv::DetectObjects>(service_name_);
    }

    // Wait briefly for service
    if (!client_->wait_for_service(200ms)) {
      RCLCPP_WARN(node_->get_logger(), "Service %s not available", service_name_.c_str());
      return BT::NodeStatus::FAILURE;
    }

    // Send async request
    auto req = std::make_shared<perseus_vision::srv::DetectObjects::Request>();
    auto far = client_->async_send_request(req);
    future_ = far.future.share();
    start_time_ = node_->now();
    RCLCPP_INFO(node_->get_logger(), "DetectAruco: calling /detect_objects");
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    double timeout_sec = 0.2;
    getInput("timeout_sec", timeout_sec);

    // Spin the internal node a little to process the future
    rclcpp::spin_some(node_);

    auto status = future_.wait_for(0ms);
    if (status != std::future_status::ready) {
      // timeout?
      auto elapsed = (node_->now() - start_time_).seconds();
      if (elapsed > timeout_sec) {
        RCLCPP_WARN(node_->get_logger(), "DetectObjects timed out");
        return BT::NodeStatus::FAILURE;
      }
      return BT::NodeStatus::RUNNING;
    }

    // Got response
    auto resp = future_.get();

    // Freshness check
    double max_age_sec = 0.5;
    getInput("max_age_sec", max_age_sec);

    const auto now = node_->now();
    const rclcpp::Time stamp(resp->stamp.sec, resp->stamp.nanosec, node_->get_clock()->get_clock_type());
    const double age = (now - stamp).seconds();

    if (age > max_age_sec) {
      RCLCPP_WARN(node_->get_logger(), "Detections too old: age=%.3f > %.3f", age, max_age_sec);
      return BT::NodeStatus::FAILURE;
    }

    // Copy outputs to blackboard
    std::vector<int> ids(resp->ids.begin(), resp->ids.end());
    std::vector<geometry_msgs::msg::Pose> poses(resp->poses.begin(), resp->poses.end());

    setOutput("ids", ids);
    setOutput("poses", poses);
    setOutput("frame_id", resp->frame_id);

    int min_count = 1;
    getInput("min_count", min_count);

    // Decide success condition
    if ((int)ids.size() >= min_count) {
      RCLCPP_INFO(node_->get_logger(),
        "DetectAruco: SUCCESS (%zu detections)",
        ids.size());
      return BT::NodeStatus::SUCCESS;

    }
    return BT::NodeStatus::FAILURE;
    RCLCPP_INFO(node_->get_logger(),
      "DetectAruco: FAILURE (no valid detections)");
  }

  void onHalted() override
  {
    // Nothing special
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<perseus_vision::srv::DetectObjects>::SharedPtr client_;
  std::string service_name_;

  rclcpp::Time start_time_;
  rclcpp::Client<perseus_vision::srv::DetectObjects>::SharedFuture future_;
};
}  // namespace perseus_bt_nodes
