#pragma once

#include <string>

#include "behaviortree_cpp/action_node.h"

namespace perseus_bt_nodes
{

    class IncrementIndex : public BT::SyncActionNode
    {
    public:
        IncrementIndex(const std::string& name, const BT::NodeConfiguration& config)
            : BT::SyncActionNode(name, config)
        {
        }

        static BT::PortsList providedPorts()
        {
            return {
                BT::BidirectionalPort<int>("index")};
        }

        BT::NodeStatus tick() override
        {
            int index = 0;
            if (!getInput("index", index))
            {
                throw BT::RuntimeError("IncrementIndex: missing input [index]");
            }

            index += 1;
            setOutput("index", index);
            return BT::NodeStatus::SUCCESS;
        }
    };

}  // namespace perseus_bt_nodes
