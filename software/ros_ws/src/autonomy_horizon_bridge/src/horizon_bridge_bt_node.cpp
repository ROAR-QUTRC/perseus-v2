#include <nav2_behavior_tree/bt_action_node.hpp>
#include <nav2_msgs/action/compute_path_to_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "perseus_msgs/srv/get_frontier_goal.hpp"

class HorizonBridge : public BT::SyncActionNode
{
public:
  HorizonBridge(
    const std::string & name,
    const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config)
  {
    node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    planner_client_ =
      rclcpp_action::create_client<nav2_msgs::action::ComputePathToPose>(
        node_, "compute_path_to_pose");

    frontier_client_ =
      node_->create_client<perseus_msgs::srv::GetFrontierGoal>(
        "/horizon_bridge/get_frontier_goal");
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("original_goal"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("nav_goal")
    };
  }

  BT::NodeStatus tick() override
  {
    geometry_msgs::msg::PoseStamped original;
    getInput("original_goal", original);

    // Try to plan to original goal
    nav2_msgs::action::ComputePathToPose::Goal goal;
    goal.goal = original;

    if (!planner_client_->wait_for_action_server(std::chrono::milliseconds(200))) {
      RCLCPP_WARN(node_->get_logger(), "Planner not available");
    } else {
      auto future = planner_client_->async_send_goal(goal);
      if (rclcpp::spin_until_future_complete(node_, future, std::chrono::milliseconds(300))
          == rclcpp::FutureReturnCode::SUCCESS)
      {
        setOutput("nav_goal", original);
        return BT::NodeStatus::SUCCESS;
      }
    }

    // Planner failed → get frontier
    auto req = std::make_shared<perseus_msgs::srv::GetFrontierGoal::Request>();
    req->original_goal = original;

    if (!frontier_client_->wait_for_service(std::chrono::milliseconds(300))) {
      RCLCPP_ERROR(node_->get_logger(), "Frontier service unavailable");
      return BT::NodeStatus::FAILURE;
    }

    auto future = frontier_client_->async_send_request(req);
    if (rclcpp::spin_until_future_complete(node_, future)
        != rclcpp::FutureReturnCode::SUCCESS)
    {
      return BT::NodeStatus::FAILURE;
    }

    auto res = future.get();
    if (!res->success) return BT::NodeStatus::FAILURE;

    setOutput("nav_goal", res->frontier_goal);
    return BT::NodeStatus::SUCCESS;
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<nav2_msgs::action::ComputePathToPose>::SharedPtr planner_client_;
  rclcpp::Client<perseus_msgs::srv::GetFrontierGoal>::SharedPtr frontier_client_;
};

#include <behaviortree_cpp/bt_factory.h>
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<HorizonBridge>("HorizonBridge");
}