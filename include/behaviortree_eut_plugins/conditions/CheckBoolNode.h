#ifndef GET_CHECK_BOOL_NODE_HPP
#define GET_CHECK_BOOL_NODE_HPP

#include "behaviortree_cpp/condition_node.h"

namespace BT
{
class CheckBoolNode final : public BT::ConditionNode
{
    public:
        using BT::ConditionNode::ConditionNode;
        ~CheckBoolNode() = default;

        static BT::PortsList providedPorts()
        {
            return { BT::InputPort<bool>("input", "Input boolean value") };
        }

        virtual BT::NodeStatus tick() override
        {
            const auto& bool_value = getInput<bool>("input");

            if(!bool_value) { throw BT::RuntimeError { name() + ": " + bool_value.error() }; }

            return bool_value.value() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
        }

};

}
#endif