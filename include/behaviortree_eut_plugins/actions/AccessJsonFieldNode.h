#ifndef ACCESS_JSON_FIELD_HPP
#define ACCESS_JSON_FIELD_HPP

#include <behaviortree_cpp/action_node.h>
#include "behaviortree_eut_plugins/utils/deserialize_json.h"


namespace BT
{
class AccessJsonFieldNode final : public BT::SyncActionNode
{
    public:
        using BT::SyncActionNode::SyncActionNode;
        ~AccessJsonFieldNode() = default;

        static BT::PortsList providedPorts()
        {
            //Seting void as the port type disables type checking
            return { BT::InputPort<nlohmann::json>("input", "Serialized ROS message"),
                     BT::InputPort<std::string>("field", "Field to fetch"),
                     BT::OutputPort("output", "Output variable")
                   };
        }

        virtual BT::NodeStatus tick() override
        {
            const auto& input = getInput<nlohmann::json>("input");
            const auto& field = getInput<std::string>("field");

            if(!field) { throw BT::RuntimeError { name() + ": " + field.error() }; }
            if(!input) { return BT::NodeStatus::FAILURE; }

            try
            {
                nlohmann::json::json_pointer pointer(field.value().data());
                const nlohmann::json& json_value = input.value().at(pointer);

                BT::EutUtils::deserializeField(*this, "output", json_value);
                return BT::NodeStatus::SUCCESS;
            }
            catch(const nlohmann::json::exception& e) 
            { 
                std::cerr << "AccessJsonField FAILURE: " << e.what() << std::endl;
                return BT::NodeStatus::FAILURE; 
            }
            catch(const BT::RuntimeError& e) 
            { 
                std::cerr << "AccessJsonField FAILURE, BT::RuntimeError: " << e.what() << std::endl;
                return BT::NodeStatus::FAILURE; 
            }
            catch(const BT::LogicError& e) 
            { 
                std::cerr << "AccessJsonField FAILURE, BT::LogicError:  " << e.what() << std::endl;
                return BT::NodeStatus::FAILURE; 
            }
            catch(const std::out_of_range& e)
            { 
                std::cerr << "AccessJsonField FAILURE: " << e.what() << std::endl;
                return BT::NodeStatus::FAILURE; 
            }
        }
};
}

#endif
