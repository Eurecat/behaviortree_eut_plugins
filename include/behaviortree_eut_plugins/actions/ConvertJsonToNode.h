#ifndef CONVERT_JSON_TO_NODE_HPP
#define CONVERT_JSON_TO_NODE_HPP

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_eut_plugins/utils/eut_utils.h"
//#include "behavior_tree_ros/details/conversion_json.hpp"

namespace BT
{
template <typename T>
class ConvertJsonToNode final : public BT::SyncActionNode
{
    public:
        using BT::SyncActionNode::SyncActionNode;
        ~ConvertJsonToNode() = default;

        static BT::PortsList providedPorts()
        {
            //Setting void as the port type disables type checking
            return { BT::InputPort<nlohmann::json>("input", "Serialized ROS message"),
                     BT::OutputPort<T>("output", "Output variable")

            };
        }

        virtual BT::NodeStatus tick() override
        {
            const auto& input = getInput<nlohmann::json>("input");
            
            if(!input) { return BT::NodeStatus::FAILURE; }
   
            const nlohmann::json& json_value = input.value();
            //const BT::Any& value = BT::Any(json_value) ;
            
            setOutput("output", BT::Any(json_value).cast<T>());
            return BT::NodeStatus::SUCCESS;
            
        }
};
}

#endif