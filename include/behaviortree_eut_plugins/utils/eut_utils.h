#ifndef EUT_UTIL_H
#define EUT_UTIL_H

#include "behaviortree_cpp/basic_types.h"
#include "behaviortree_eut_plugins/eut_basic_types.h"
#include "behaviortree_eut_plugins/flatbuffers/bt_flatbuffer_helper_eut.h"
#include "behaviortree_cpp/json_export.h"

namespace BT
{   
namespace EutUtils
{
    //maps with enum or enum class as keys have a lot of issues (eg: accesing wrong entries or not compiling).
    //This ensures that a proper hash is computed. A new type is defined for ease of use.
    //Unashamedly stolen from https://stackoverflow.com/a/24847480
    struct EnumClassHash
    {
        template <typename T>
        size_t operator()(T t) const
        {
            return static_cast<size_t>(t);
        }
    };

    template <typename Key>
    using HashType = typename std::conditional<std::is_enum<Key>::value, EnumClassHash, std::hash<Key>>::type;

    template <typename Key, typename T>
    using UnorderedMap = std::unordered_map<Key, T, HashType<Key>>;


    nlohmann::json lossyJsonCompress(nlohmann::json& json);

    std::unique_ptr<BT::PortInfo> getPortInfo(const BT::TreeNode& node, const std::string& port_name);
    
    // bool missingTypeInfo(const std::type_index& type);
    
    Expected<std::string> getEntryAsString(const std::string& key,const BT::Blackboard::Ptr blackboard, const bool lossy_json_compress_output = false);

    PortsValueMap getPortValuesMap(const BT::TreeNode& node, const PortDirection& dir, const bool lossy_json_compress_output = false);

    Expected<nlohmann::json> getPortValueAsJson(const BT::TreeNode& node, const std::string& port_name, const BT::PortDirection dir = BT::PortDirection::INOUT);

    Expected<nlohmann::json> eutToJson(const Any& any);
    Expected<nlohmann::json> eutToJson(const Any& any, const std::type_index & port_type_info);
    Expected<std::string> eutToJsonString(const std::string& key,const BT::Blackboard::Ptr blackboard, const bool lossy_json_compress_output = false);
    BT::JsonExporter::ExpectedEntry eutFromJson(const nlohmann::json& source);
}
};

#endif