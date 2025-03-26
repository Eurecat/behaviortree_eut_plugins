#ifndef RANDOM_SEQUENCE_VALUE_NODE_HPP
#define RANDOM_SEQUENCE_VALUE_NODE_HPP

#include "behaviortree_cpp/action_node.h"

#include "behaviortree_eut_plugins/utils/random.h"

namespace BT
{
class RandomizeSequenceNode final : public BT::SyncActionNode
{
    public:
        using BT::SyncActionNode::SyncActionNode;
        ~RandomizeSequenceNode() = default;

        static BT::PortsList providedPorts()
        {
            return { BT::InputPort<std::string>("sequence", "Sequence to randomize"),
                     BT::InputPort<std::string>("delimiter", ";", "Sequence entries delimiter"),
                     BT::OutputPort("output", "Result random sequence entry")
                   };
        }

        virtual BT::NodeStatus tick() override
        {
            const auto& raw_sequence = getInput<std::string>("sequence");
            const auto& delimiter    = getInput<std::string>("delimiter");

            if(!raw_sequence) { throw BT::RuntimeError { name() + ": " + raw_sequence.error() }; }
            if(!delimiter)    { throw BT::RuntimeError { name() + ": " + delimiter.error() }; }

            const auto& sequence = BT::splitString(raw_sequence.value(), delimiter.value().front());
            if(sequence.empty()) { return BT::NodeStatus::FAILURE; }

            setOutput("output", sequence.at(EutUtils::getRandomNumber<uint16_t>(0, sequence.size() - 1)));

            return BT::NodeStatus::SUCCESS;
        }
};
}

#endif