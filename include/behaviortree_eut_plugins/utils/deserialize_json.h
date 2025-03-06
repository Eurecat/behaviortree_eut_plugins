#ifndef JSON_DESERIALIZATION
#define JSON_DESERIALIZATION

#include <functional>
#include <behaviortree_cpp/action_node.h>

#include "behaviortree_cpp/contrib/json.hpp"
#include "behaviortree_eut_plugins/utils/eut_utils.h"

namespace BT
{
namespace EutUtils
{
    using Json = nlohmann::json;
    using DeserializeFieldFunction = std::function<void(BT::ActionNodeBase&, const std::string&, const Json&)>;

    void deserializeFieldToSignedInt(BT::ActionNodeBase& node, const std::string& port, const Json& field)
    {
        const auto port_info_ptr = getPortInfo(node, port);
        
        bool match_not_found = false; 

        if(port_info_ptr && port_info_ptr->isStronglyTyped())
        {
            const std::type_index& type_index = port_info_ptr->type();
            
            if (type_index == typeid(short)) // shall match int16_t
                node.setOutput<short>(port, field.get<short>());
            else if(type_index == typeid(int)) // shall match int32_t
                node.setOutput<int>(port, field.get<int>());
            else if(type_index == typeid(long)) // shall match int64_t
                node.setOutput<long>(port, field.get<long>());
            else 
                match_not_found = true; //perform setOutput to max int, i.e. int64
            
        }
        else
            match_not_found = true;
        
        if(match_not_found)
        {
            // Don't have enough info (shall not happen in a good design tree)
            node.setOutput<int64_t>(port, field.get<int64_t>()); // just jump to the highest int
        }

    
    }

    void deserializeFieldToUnsignedInt(BT::ActionNodeBase& node, const std::string& port, const Json& field)
    {
        const auto port_info_ptr = getPortInfo(node, port);
        
        bool match_not_found = false; 

        if(port_info_ptr && port_info_ptr->isStronglyTyped())
        {
            const std::type_index& type_index = port_info_ptr->type();

            if(type_index == typeid(unsigned short)) // shall match uint16_t
                node.setOutput<unsigned short>(port, field.get<unsigned short>());
            else if(type_index == typeid(unsigned int)) // shall match uint32_t
                node.setOutput<unsigned int>(port, field.get<unsigned int>());
            else if(type_index == typeid(unsigned long)) // shall match uint64_t and size_t
                node.setOutput<unsigned long>(port, field.get<unsigned long>());
             else 
                match_not_found = true; //perform setOutput to max int, i.e. uint64
        }
        else
            match_not_found = true;
        
        if(match_not_found)
        {
            // Don't have enough info (shall not happen in a good design tree)
            node.setOutput<uint64_t>(port, field.get<uint64_t>()); // just jump to the highest uint
        }
    
    }

    void deserializeFieldToDouble(BT::ActionNodeBase& node, const std::string& port, const Json& field)
    {
        const auto port_info_ptr = getPortInfo(node, port);

        if(port_info_ptr && port_info_ptr->isStronglyTyped() && port_info_ptr->type() == typeid(float))
        {
            node.setOutput<float>(port, field.get<float>());
        }
        else 
            node.setOutput(port, field.get<double>());
    }

    inline void deserializeFieldToBool(BT::ActionNodeBase& node, const std::string& port, const Json& field)
    {
        node.setOutput<bool>(port, field.get<bool>());
    }

    inline void deserializeFieldToString(BT::ActionNodeBase& node, const std::string& port, const Json& field)
    {
        std::string extracted = field.get<std::string>();
        // std::cout << "deserializeFieldToString for " << port << " with value " << extracted << "\n" << std::flush;
        if (!extracted.empty() && extracted.front() == '"' && extracted.back() == '"') 
        {
            extracted = extracted.substr(1, extracted.size() - 2);
            // std::cout << "deserializeFieldToString for " << port << " with pp value " << extracted << "\n" << std::flush;
        }
        node.setOutput<std::string>(port, extracted);
    }

    inline void deserializeFieldToJson(BT::ActionNodeBase& node, const std::string& port, const Json& field)
    {
        node.setOutput<Json>(port, field);
    }

    static const UnorderedMap<Json::value_t, DeserializeFieldFunction> deserialize_field_map
    {
        { Json::value_t::boolean,         [] (auto& node, const auto& port, const auto& field) { deserializeFieldToBool(node, port, field);        }},
        { Json::value_t::string,          [] (auto& node, const auto& port, const auto& field) { deserializeFieldToString(node, port, field); }},
        { Json::value_t::number_integer,  [] (auto& node, const auto& port, const auto& field) { deserializeFieldToSignedInt(node, port, field);     }},
        { Json::value_t::number_unsigned, [] (auto& node, const auto& port, const auto& field) { deserializeFieldToUnsignedInt(node, port, field);    }},
        { Json::value_t::number_float,    [] (auto& node, const auto& port, const auto& field) { deserializeFieldToDouble(node, port, field);      }},
        { Json::value_t::object,          [] (auto& node, const auto& port, const auto& field) { deserializeFieldToJson(node, port, field);        }},
        { Json::value_t::array,           [] (auto& node, const auto& port, const auto& field) { deserializeFieldToJson(node, port, field);        }},
    };

    inline void deserializeField(BT::ActionNodeBase& node, const std::string& port, const Json& field)
    {
        deserialize_field_map.at(field.type())(node, port, field);
    }

}
}

#endif
