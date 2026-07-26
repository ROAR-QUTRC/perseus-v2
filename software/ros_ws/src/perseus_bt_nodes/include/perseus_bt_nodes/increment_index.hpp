#pragma once

/// @file increment_index.hpp
/// @brief Behaviour tree action node which increments an integer blackboard entry.

#include <behaviortree_cpp/action_node.h>

#include <string>

namespace perseus_bt_nodes
{
    /// @brief Adds one to an integer held on the blackboard.
    ///
    /// Pairs with GetGoalFromGoals to walk a goal list one entry per loop iteration.
    ///
    /// @note `providedPorts` and `tick` are named in camelCase because they override the
    ///       BehaviorTree.CPP interface, which mandates those names.
    class IncrementIndex : public BT::SyncActionNode
    {
    private:
        static constexpr const char* INDEX_PORT = "index";

    public:
        /// @brief Constructs the node.
        /// @param name Instance name given to this node by the behaviour tree.
        /// @param config Port remapping and blackboard configuration supplied by the factory.
        IncrementIndex(const std::string& name, const BT::NodeConfiguration& config);

        /// @brief Declares the bidirectional index port exposed to the behaviour tree XML.
        /// @return The index port, which is both read and written.
        static BT::PortsList providedPorts();

        /// @brief Reads the index, increments it by one, and writes it back.
        /// @return Always SUCCESS.
        /// @throws BT::RuntimeError If the index port is unset.
        BT::NodeStatus tick() override;
    };

    inline IncrementIndex::IncrementIndex(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config)
    {
    }

    inline BT::PortsList IncrementIndex::providedPorts()
    {
        return {
            BT::BidirectionalPort<int>(INDEX_PORT)};
    }

    inline BT::NodeStatus IncrementIndex::tick()
    {
        int index = 0;
        if (!getInput(INDEX_PORT, index))
        {
            throw BT::RuntimeError("IncrementIndex: missing input [index]");
        }

        ++index;
        setOutput(INDEX_PORT, index);
        return BT::NodeStatus::SUCCESS;
    }

}  // namespace perseus_bt_nodes
