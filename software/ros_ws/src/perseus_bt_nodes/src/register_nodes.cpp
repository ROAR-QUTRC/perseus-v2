#include <memory>

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/utils/shared_library.h"
#include "perseus_bt_nodes/get_goal_from_goals.hpp"
#include "perseus_bt_nodes/increment_index.hpp"
#include "perseus_bt_nodes/pop_first_goal.hpp"
// #include "perseus_bt_nodes/detect_aruco_bt_node.hpp"

extern "C" void BT_RegisterNodesFromPlugin(BT::BehaviorTreeFactory& factory)
{
  factory.registerNodeType<perseus_bt_nodes::GetGoalFromGoals>("GetGoalFromGoals");
  factory.registerNodeType<perseus_bt_nodes::IncrementIndex>("IncrementIndex");
  factory.registerNodeType<perseus_bt_nodes::PopFirstGoal>("PopFirstGoal");
  // factory.registerNodeType<perseus_bt_nodes::DetectAruco>("DetectAruco");
}
