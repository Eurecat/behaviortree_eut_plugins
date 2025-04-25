#ifndef ADD_JSON_TO_ARRAY
#define ADD_JSON_TO_ARRAY

#include <behaviortree_cpp/action_node.h>

namespace BT
{
class AddJsonToArray final : public BT::SyncActionNode
{
    public:
        using BT::SyncActionNode::SyncActionNode;
        ~AddJsonToArray() = default;

        static BT::PortsList providedPorts()
        {
            return { BT::InputPort<bool>("override", true, "1 to override the actual array; 0 to append value"),
                     BT::InputPort<std::string>("input_key", "", "Key name of the array, leave empty to just push"),
                     BT::InputPort("input_json", "New value or value to append to the array"),
                     BT::InputPort("input_array", "Input Json array"),
                     BT::OutputPort<nlohmann::json>("output_json", "Output Json array")
            };
        }

        virtual BT::NodeStatus tick() override
        {
            const auto& override = getInput<bool>("override");
            const auto& input_key = getInput<std::string>("input_key");
            const auto& input_value = getInput<nlohmann::json>("input_json");
            const auto& input_json = getInput<nlohmann::json>("input_array");

            if (!override) { throw BT::RuntimeError { name() + ": " + override.error() }; }
            if (!input_value) { throw BT::RuntimeError { name() + ": " + input_value.error() }; }
            if (!input_json) { throw BT::RuntimeError { name() + ": " + input_json.error() }; }

            try
            {
                // Check if input value has json format in case it's an array or object already
                const nlohmann::json& input_value_json = input_value.value();

                // Check if input Json is empty
                // If you try to parse an empty variable an exception is thrown
                nlohmann::json output_json;
                if (!input_json.value().empty()) { output_json = input_json.value(); }

                if(output_json.is_object()) // Json is an object
                {
                    // Clear array elem if we have to override it
                    if (override.value()) { output_json[input_key.value()].clear(); }

                    output_json[input_key.value()].push_back(input_value_json);
                }
                else if(output_json.is_array()) // Json is an array
                {
                    if (input_key.value_or("") != "") // Add to array with key
                    {
                        bool filled = false;
                        // Check all the elements of the array for an object with the same key
                        for(auto& elem : output_json)
                            if(elem.is_object())
                                if(elem.find(input_key.value()) != elem.end())
                                {
                                    const nlohmann::json::iterator& key_elem = elem.find(input_key.value());

                                    if (override.value()) { key_elem.value().clear(); }

                                    key_elem.value().push_back(input_value_json);

                                    filled = true;
                                    break;
                                }
                        // If there was no object, create a new object with the array
                        if(!filled)
                        {
                            nlohmann::json object;
                            object[input_key.value()].push_back(input_value_json);
                            output_json.push_back(object);
                        }
                    }
                    else // Add to array without key
                    {
                        if (override.value()) { output_json.clear(); }

                        output_json.push_back(input_value_json);
                    }
                }
                else // Empty Json
                {
                    if (input_key.value_or("") != "")
                    {
                        output_json[input_key.value()].push_back(input_value_json);
                    }
                    else
                    {
                        output_json.push_back(input_value_json);
                    }
                }

                setOutput("output_json", output_json);
                return  BT::NodeStatus::SUCCESS;
            }
            catch (const nlohmann::json::exception& ex) { return BT::NodeStatus::FAILURE; }
        }
};
}

#endif