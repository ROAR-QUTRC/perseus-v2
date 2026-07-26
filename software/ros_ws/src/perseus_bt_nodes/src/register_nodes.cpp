/// @file register_nodes.cpp
/// @brief BehaviorTree.CPP plugin entry point registering the Perseus behaviour tree nodes.
///
/// Built as a shared library and loaded by the navigation behaviour tree engine, which
/// calls BT_RegisterNodesFromPlugin() to learn the node types this package provides.

#include <behaviortree_cpp/bt_factory.h>

#include "perseus_bt_nodes/get_goal_from_goals.hpp"
#include "perseus_bt_nodes/increment_index.hpp"
#include "perseus_bt_nodes/pop_first_goal.hpp"

/// @brief Registers every Perseus behaviour tree node with the loading factory.
/// @param factory Factory supplied by the behaviour tree engine loading this plugin.
///
/// The registered names are the element names used in the behaviour tree XML files
/// under `autonomy/behavior_trees/`.
extern "C" void BT_RegisterNodesFromPlugin(BT::BehaviorTreeFactory& factory)
{
    factory.registerNodeType<perseus_bt_nodes::GetGoalFromGoals>("GetGoalFromGoals");
    factory.registerNodeType<perseus_bt_nodes::IncrementIndex>("IncrementIndex");
    factory.registerNodeType<perseus_bt_nodes::PopFirstGoal>("PopFirstGoal");
}
