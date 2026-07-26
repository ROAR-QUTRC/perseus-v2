#pragma once

/// @file get_goal_from_goals.hpp
/// @brief Behaviour tree action node which selects one goal out of a goal list by index.

#include <behaviortree_cpp/action_node.h>

#include <cstddef>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <string>
#include <vector>

namespace perseus_bt_nodes
{
    /// @brief Reads a single goal pose out of a goal list at a given index.
    ///
    /// Intended to be driven by a loop decorator alongside IncrementIndex: the node
    /// returns FAILURE once the index runs past the end of the list, which terminates
    /// a `RepeatUntilFailure` loop cleanly rather than throwing.
    ///
    /// @note `providedPorts` and `tick` are named in camelCase because they override the
    ///       BehaviorTree.CPP interface, which mandates those names.
    class GetGoalFromGoals : public BT::SyncActionNode
    {
    private:
        static constexpr const char* GOALS_PORT = "goals";
        static constexpr const char* INDEX_PORT = "index";
        static constexpr const char* GOAL_PORT = "goal";

    public:
        /// @brief Constructs the node.
        /// @param name Instance name given to this node by the behaviour tree.
        /// @param config Port remapping and blackboard configuration supplied by the factory.
        GetGoalFromGoals(const std::string& name, const BT::NodeConfiguration& config);

        /// @brief Declares the input and output ports exposed to the behaviour tree XML.
        /// @return The goal list and index inputs, and the selected goal output.
        static BT::PortsList providedPorts();

        /// @brief Writes the goal at the requested index to the output port.
        /// @return SUCCESS if the index is in range, FAILURE once it runs past the end.
        /// @throws BT::RuntimeError If either input port is unset.
        BT::NodeStatus tick() override;
    };

    inline GetGoalFromGoals::GetGoalFromGoals(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config)
    {
    }

    inline BT::PortsList GetGoalFromGoals::providedPorts()
    {
        return {
            BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>(GOALS_PORT),
            BT::InputPort<int>(INDEX_PORT),
            BT::OutputPort<geometry_msgs::msg::PoseStamped>(GOAL_PORT)};
    }

    inline BT::NodeStatus GetGoalFromGoals::tick()
    {
        std::vector<geometry_msgs::msg::PoseStamped> goals;
        int index = 0;

        if (!getInput(GOALS_PORT, goals))
        {
            throw BT::RuntimeError("GetGoalFromGoals: missing input [goals]");
        }
        if (!getInput(INDEX_PORT, index))
        {
            throw BT::RuntimeError("GetGoalFromGoals: missing input [index]");
        }

        if (index < 0 || static_cast<std::size_t>(index) >= goals.size())
        {
            // Important: returning FAILURE is useful to stop a RepeatUntilFailure loop.
            return BT::NodeStatus::FAILURE;
        }

        setOutput(GOAL_PORT, goals[static_cast<std::size_t>(index)]);
        return BT::NodeStatus::SUCCESS;
    }

}  // namespace perseus_bt_nodes
