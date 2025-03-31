#ifndef ADD_KEY_VALUE_TO_NODE
#define ADD_KEY_VALUE_TO_NODE

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/utils/safe_any.hpp>
#include <behaviortree_eut_plugins/utils/eut_utils.h>

namespace BT
{
class AddKeyValueToJson final : public BT::SyncActionNode
{
    public:
        using BT::SyncActionNode::SyncActionNode;
        ~AddKeyValueToJson() = default;

        static BT::PortsList providedPorts()
        {
            return { BT::InputPort<std::string>("input_key", "Input key name"),
                     BT::InputPort<void>("input_value", "Input value"),
                     BT::InputPort<nlohmann::json>("input_json", "Input json to copy to"),
                     BT::OutputPort<nlohmann::json>("output", "Output json with new value") };
        }

        virtual BT::NodeStatus tick() override
        {
            const auto& input_key = getInput<std::string>("input_key");

            //TODO: Check if this is the correct way to get the value
            //const auto& input_value = getInputAsJson("input_value");
            const auto& input_value = BT::EutUtils::getPortValueAsJson(*this, "input_value", BT::PortDirection::INPUT);
            //
            
            const auto& input_json = getInput<nlohmann::json>("input_json");
            if(!input_key) { throw BT::RuntimeError { name() + ": missing \"input_key\" required fields. Error: " + input_key.error() }; }
            if(!input_value) { throw BT::RuntimeError { name() + ": missing \"input_value\" required fields. Error: " + input_value.error() }; }
            if(!input_json) { throw BT::RuntimeError { name() + ": missing \"input_json\" required fields. Error: " + input_json.error() }; }
            
            try{
                nlohmann::json json_store = input_json.value();

                // Fill Json according to its type
                if (json_store.is_array())
                {
                    nlohmann::json object;
                    object[input_key.value()] = input_value.value();
                    json_store.push_back(object);
                }
                else
                    json_store[input_key.value()] = input_value.value();

                setOutput("output", json_store);
                return BT::NodeStatus::SUCCESS;
            }catch(const nlohmann::json::exception&) {
                std::cout << "Error when updating JSON"; 
                return BT::NodeStatus::FAILURE; }
        }
};

}

#endif