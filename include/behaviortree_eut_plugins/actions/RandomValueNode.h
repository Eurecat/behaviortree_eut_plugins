#ifndef RANDOM_VALUE_NODE_HPP
#define RANDOM_VALUE_NODE_HPP

#include "behaviortree_cpp/action_node.h"

#include "behaviortree_eut_plugins/utils/random.h"

namespace BT
{
template <class T>
class RandomValueNode final : public BT::SyncActionNode
{
    public:
        using BT::SyncActionNode::SyncActionNode;
        ~RandomValueNode() = default;

        static BT::PortsList providedPorts()
        {
            return { BT::InputPort<T>("min", "Minimum range value"),
                     BT::InputPort<T>("max", "Max range value"),
                     BT::OutputPort<T>("result", "Random result value")
                   };
        }

        virtual BT::NodeStatus tick() override
        {
            const auto& min = getInput<T>("min");
            const auto& max = getInput<T>("max");

            if(!min) { throw BT::RuntimeError { name() + ": " + min.error() }; }
            if(!max) { throw BT::RuntimeError { name() + ": " + max.error() }; }

            if(min.value() > max.value()) { throw BT::RuntimeError { name() + " : max value shall be greater than min." }; }

            setOutput("result", EutUtils::getRandomNumber<T>(min.value(), max.value()));
            return BT::NodeStatus::SUCCESS;
        }
};
}

#endif
