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
                
                auto output_key = (this->config().output_ports.find("output") != this->config().output_ports.end()) ?
                                        this->config().output_ports.at("output") : "output";
                if(BT::TreeNode::isBlackboardPointer(output_key))
                {
                    output_key = BT::TreeNode::stripBlackboardPointer(output_key);
                }

                auto entry = this->config().blackboard->getEntry(output_key);
                bool success_updated = false;
                std::chrono::nanoseconds last_entry_updated{0};
                if(entry)
                {
                    std::lock_guard<std::mutex> lock(entry->entry_mutex);
                    last_entry_updated = entry->stamp;
                }
                BT::EutUtils::deserializeField(*this, "output", json_value); // this might silently fail the update (no exception)
                entry = this->config().blackboard->getEntry(output_key); // re-fetch entry to get updated stamp (it might have been created now)
                if(entry)
                {
                    std::lock_guard<std::mutex> lock(entry->entry_mutex);
                    success_updated = (last_entry_updated < entry->stamp);
                }

                return (success_updated) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
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
            catch(const std::exception& e)
            { 
                std::cerr << "AccessJsonField FAILURE exception: " << e.what() << std::endl;
                return BT::NodeStatus::FAILURE; 
            }
        }
};
}

#endif
