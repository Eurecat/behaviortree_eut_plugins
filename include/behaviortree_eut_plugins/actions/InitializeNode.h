#ifndef INITIALIZE_NODE_HPP
#define INITIALIZE_NODE_HPP

#include <behaviortree_cpp/action_node.h>

namespace BT
{
template <class T>
class InitializeNode final : public BT::SyncActionNode
{
    public:
        using BT::SyncActionNode::SyncActionNode;
        ~InitializeNode() = default;

        static BT::PortsList providedPorts()
        {
            return { BT::OutputPort<T>("output", "Output variable to initialize") };
        }

        virtual BT::NodeStatus tick() override
        {
            setOutput("output", T{});

            return BT::NodeStatus::SUCCESS;
        }
};
}

#endif