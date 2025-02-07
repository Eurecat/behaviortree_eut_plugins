#ifndef EUT_UTIL_H
#define EUT_UTIL_H

#include "behaviortree_cpp/basic_types.h"
#include "behaviortree_eut_plugins/eut_basic_types.h"
#include "behaviortree_eut_plugins/flatbuffers/bt_flatbuffer_helper_eut.h"
#include "behaviortree_cpp/json_export.h"

namespace BT
{   

    nlohmann::json lossyJsonCompress(nlohmann::json& json);
    
    // bool missingTypeInfo(const std::type_index& type);
    
    Expected<std::string> getEntryAsString(const std::string& key,const BT::Blackboard::Ptr blackboard, const bool lossy_json_compress_output = false);

    PortsValueMap getPortValuesMap(const BT::TreeNode& node, const PortDirection& dir, const bool lossy_json_compress_output = false);

    Expected<nlohmann::json> getPortValueAsJson(const BT::TreeNode& node, const std::string& port_name, const BT::PortDirection dir = BT::PortDirection::INOUT);

    Expected<nlohmann::json> eutToJson(const Any& any);
    Expected<nlohmann::json> eutToJson(const Any& any, const std::type_index & type_info);
    Expected<std::string> eutToJsonString(const std::string& key,const BT::Blackboard::Ptr blackboard);
    BT::JsonExporter::ExpectedEntry eutFromJson(const nlohmann::json& source);

};

#endif