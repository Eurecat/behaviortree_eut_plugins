#ifndef GET_CHECK_EMPTY_JSON_NODE_HPP
#define GET_CHECK_EMPTY_JSON_NODE_HPP

#include "behaviortree_cpp/condition_node.h"

namespace BT
{
class CheckEmptyJson final : public BT::ConditionNode
{
    public:
        using BT::ConditionNode::ConditionNode;
        ~CheckEmptyJson() = default;

        static BT::PortsList providedPorts()
        {
            std::cout << "CheckEmptyJson::providedPorts loading converter for type " << BT::demangle(typeid(nlohmann::json)) << "\n" << std::flush;
            return { BT::InputPort<nlohmann::json>("input", "Input json value") };
        }

        virtual BT::NodeStatus tick() override
        {
            const auto& json = getInput<nlohmann::json>("input");

            if(!json) { throw BT::RuntimeError { name() + ": " + json.error() }; }

            return json.value().empty() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
        }

};

}
#endif