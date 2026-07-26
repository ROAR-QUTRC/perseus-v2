#pragma once

/// @file pop_first_goal.hpp
/// @brief Behaviour tree action node which drops the leading goal from a goal list.

#include <behaviortree_cpp/action_node.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <string>
#include <vector>

namespace perseus_bt_nodes
{
    /// @brief Copies a goal list to the output port with its first entry removed.
    ///
    /// Lets a tree consume a goal queue destructively without mutating the input port.
    /// An empty input list is not an error - it is passed through unchanged.
    ///
    /// @note `providedPorts` and `tick` are named in camelCase because they override the
    ///       BehaviorTree.CPP interface, which mandates those names.
    class PopFirstGoal : public BT::SyncActionNode
    {
    private:
        static constexpr const char* INPUT_GOALS_PORT = "input_goals";
        static constexpr const char* OUTPUT_GOALS_PORT = "output_goals";

    public:
        /// @brief Constructs the node.
        /// @param name Instance name given to this node by the behaviour tree.
        /// @param config Port remapping and blackboard configuration supplied by the factory.
        PopFirstGoal(const std::string& name, const BT::NodeConfiguration& config);

        /// @brief Declares the input and output goal list ports exposed to the behaviour tree XML.
        /// @return The input goal list and the remaining-goals output.
        static BT::PortsList providedPorts();

        /// @brief Writes the input goal list, minus its first entry, to the output port.
        /// @return Always SUCCESS, including when the input list is empty.
        /// @throws BT::RuntimeError If the input goal list port is unset.
        BT::NodeStatus tick() override;
    };

    inline PopFirstGoal::PopFirstGoal(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config)
    {
    }

    inline BT::PortsList PopFirstGoal::providedPorts()
    {
        return {
            BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>(INPUT_GOALS_PORT),
            BT::OutputPort<std::vector<geometry_msgs::msg::PoseStamped>>(OUTPUT_GOALS_PORT)};
    }

    inline BT::NodeStatus PopFirstGoal::tick()
    {
        std::vector<geometry_msgs::msg::PoseStamped> goals;

        if (!getInput(INPUT_GOALS_PORT, goals))
        {
            throw BT::RuntimeError("PopFirstGoal: missing input [input_goals]");
        }

        // If there are goals, remove the first one
        if (!goals.empty())
        {
            goals.erase(goals.begin());
        }

        // Set the output with remaining goals
        setOutput(OUTPUT_GOALS_PORT, goals);
        return BT::NodeStatus::SUCCESS;
    }

}  // namespace perseus_bt_nodes
