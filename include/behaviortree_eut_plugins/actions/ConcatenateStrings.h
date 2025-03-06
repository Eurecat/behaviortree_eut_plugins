#ifndef CONCATENATE_STRINGS_NODE_HPP
#define CONCATENATE_STRINGS_NODE_HPP

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_eut_plugins/utils/eut_utils.h"

namespace BT
{
template <size_t NUM_STRINGS>
class ConcatenateStringsNode final : public BT::SyncActionNode
{
    public:
        using BT::SyncActionNode::SyncActionNode;
        ~ConcatenateStringsNode() = default;

        static BT::PortsList providedPorts()
        {
            BT::PortsList ports;
            ports.insert( BT::InputPort("first", "1st item to be taken as string and concatenated") );
            ports.insert( BT::InputPort("second", "2nd item to be taken as string and concatenated") );

            std::string port_name;
            for (unsigned i=3; i <= NUM_STRINGS; i++)
            {
                port_name = "string_" + std::to_string(i);
                ports.insert( BT::InputPort(port_name,  std::to_string(i) + "th item to be taken as string and concatenated" ));
            }

            ports.insert( BT::OutputPort<std::string>("output", "Concatenated result string") );
            return ports;
        }

        virtual BT::NodeStatus tick() override
        {
            auto trimQuotes = [](std::string& str) {
                if (str.size() >= 2 && (str.front() == '"' || str.front() == '\'') && str.front() == str.back()) {
                    str.erase(0, 1);         // Remove first character
                    str.pop_back();          // Remove last character
                }
            };

            const auto& first  = EutUtils::getPortValueAsJson(*this, "first", BT::PortDirection::INPUT);
            const auto& second = EutUtils::getPortValueAsJson(*this, "second", BT::PortDirection::INPUT);

            if(!first.has_value())  { throw BT::RuntimeError { name() + ": " + first.error()  }; }
            if(!second.has_value()) { throw BT::RuntimeError { name() + ": " + second.error() }; }

            std::string str1 = first.value().dump(); trimQuotes(str1);
            std::string str2 = second.value().dump(); trimQuotes(str2);
            
            std::string output_string = str1 + str2;

            std::string port_name;
            Expected<nlohmann::json> opt_value;
            // bool found = false;
            for (unsigned i=3; i <= NUM_STRINGS; i++)
            {
                port_name = "string_" + std::to_string(i);
                opt_value = EutUtils::getPortValueAsJson(*this, port_name, BT::PortDirection::INPUT);
                if(!opt_value.has_value()) { throw BT::RuntimeError { name() + ": " + second.error() }; }

                std::string tmp_str = opt_value.value().dump();
                trimQuotes(tmp_str);
                output_string += tmp_str;
            }

            setOutput("output", output_string);
            return BT::NodeStatus::SUCCESS;
        }
};
}

#endif
