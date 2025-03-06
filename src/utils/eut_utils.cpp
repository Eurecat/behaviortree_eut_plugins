#include "behaviortree_eut_plugins/utils/eut_utils.h"


#define JSON_TRUNC_STR_DIM 16

namespace BT
{
namespace EutUtils
{
    // bool missingTypeInfo(const std::type_index& type)
    // {
    //     return type == typeid(BT::Any) || type == typeid(void); // TODO or AnyTypeAllowed??
    // }


    std::unique_ptr<BT::PortInfo> getPortInfo(const BT::TreeNode& node, const std::string& port_name)
    {
        if(!node.config().manifest)
        {
            return nullptr;
        }
        else
        {
            // maybe it is declared with a default value in the manifest
            auto port_manifest_it = node.config().manifest->ports.find(port_name);
            if(port_manifest_it == node.config().manifest->ports.end())
            {
                return nullptr;
            }
            return std::make_unique<BT::PortInfo>(port_manifest_it->second.direction(), port_manifest_it->second.type(), port_manifest_it->second.converter());
        }
    }

    static
    void fixTypeMismatchJson(nlohmann::json& json, const std::type_index& info)
    {
        if(info == typeid(bool))
            json = BT::convertFromString<bool>(json.dump());
    }

    nlohmann::json lossyJsonCompress(nlohmann::json& json)
    {
        if (json.is_object()) 
        {
            for (auto it = json.begin(); it != json.end(); ++it) 
            {
                *it = lossyJsonCompress(it.value());
            }
        } 
        else if (json.is_array()) 
        {
            for (auto& element : json) 
            {
                element = lossyJsonCompress(element);
            }
        } 
        else if (json.is_string()) 
        {
            std::string str = json.get<std::string>();
            if (str.size() > JSON_TRUNC_STR_DIM) 
            {
                str = str.substr(0, JSON_TRUNC_STR_DIM); // Truncate to 16 characters
            }
            return nlohmann::json{str};
        } 
        else if (json.is_number_float()) 
        {
            return nlohmann::json{std::round(json.get<double>() * 1000.0) / 1000.0}; // Limit to 3 decimal places
        }
        
        // Return the input unchanged for other types
        return json;
    }

    Expected<nlohmann::json> eutToJson(const Any& any)
    {
        return BT::EutUtils::eutToJson(any, typeid(void));
    }

    Expected<nlohmann::json> eutToJson(const Any& any, const std::type_index& port_type_info)
    {
        nlohmann::json json;
        if(any.type() == typeid(nlohmann::json))
        {
            return any.tryCast<nlohmann::json>();
        }

        else if(!isStronglyTyped(any.type()) && any.isString())
        {
            return nlohmann::json{any.cast<std::string>()};
        }

        else if(JsonExporter::get().toJson(any, json))
        {
            fixTypeMismatchJson(json, port_type_info);
            return json;
        }
        else
            return nonstd::make_unexpected(StrCat("Cannot convert any of type [",BT::demangle(any.type()),"] to a Json object in eutToJson"));
    }


    Expected<std::string> eutToJsonString(const std::string& key,const BT::Blackboard::Ptr blackboard, const bool lossy_json_compress_output)
    {
        if(blackboard)
        {
            if(auto any_locked_ptr = blackboard->getAnyLocked(key))
            {   
                BT::Any any; any_locked_ptr->copyInto(any);
                if(!isStronglyTyped(any.type()) && any.isString())
                {
                    return nlohmann::json{any.cast<std::string>()};
                }
                else if(auto json_expected = eutToJson(any))
                {
                    auto& json = json_expected.value();
                    return lossy_json_compress_output? lossyJsonCompress(json).dump() : json.dump();
                }
                else
                    return nonstd::make_unexpected(json_expected.error());;
            }
        }

        return nonstd::make_unexpected(StrCat("Missing entry for key [",key,"] while performing a eutToJsonString operation"));
    }

    BT::JsonExporter::ExpectedEntry eutFromJson(const nlohmann::json& source, const std::type_index type)
    {
        BT::JsonExporter::ExpectedEntry expected_entry = JsonExporter::get().fromJson(source);
        if(!expected_entry.has_value() && ( source.is_object() || source.is_array()))
         return BT::JsonExporter::Entry{ BT::Any(source), BT::TypeInfo::Create<nlohmann::json>() };
        
        if(expected_entry.has_value())
        {
            BT::JsonExporter::Entry& entry = expected_entry.value();
            if(entry.second.type() != type)
            {
                //fix type inconsistency
                if(source.is_number() && isNumberType(type))
                {
                    if(std::type_index(typeid(uint8_t)) == type)
                    {
                        return BT::JsonExporter::Entry{ BT::Any(source.get<uint8_t>()), BT::TypeInfo::Create<uint8_t>() };
                    }
                    if(std::type_index(typeid(uint16_t)) == type)
                    {
                        return BT::JsonExporter::Entry{ BT::Any(source.get<uint16_t>()), BT::TypeInfo::Create<uint16_t>() };
                    }
                    if(std::type_index(typeid(uint32_t)) == type)
                    {
                        return BT::JsonExporter::Entry{ BT::Any(source.get<uint32_t>()), BT::TypeInfo::Create<uint32_t>() };
                    }
                    if(std::type_index(typeid(uint64_t)) == type)
                    {
                        return BT::JsonExporter::Entry{ BT::Any(source.get<uint64_t>()), BT::TypeInfo::Create<uint64_t>() };
                    }
                    //------------
                    if(std::type_index(typeid(int8_t)) == type)
                    {
                        return BT::JsonExporter::Entry{ BT::Any(source.get<int8_t>()), BT::TypeInfo::Create<int8_t>() };
                    }
                    if(std::type_index(typeid(int16_t)) == type)
                    {
                        return BT::JsonExporter::Entry{ BT::Any(source.get<int16_t>()), BT::TypeInfo::Create<int16_t>() };
                    }
                    if(std::type_index(typeid(int32_t)) == type)
                    {
                        return BT::JsonExporter::Entry{ BT::Any(source.get<int32_t>()), BT::TypeInfo::Create<int32_t>() };
                    }
                    if(std::type_index(typeid(int64_t)) == type)
                    {
                        return BT::JsonExporter::Entry{ BT::Any(source.get<int64_t>()), BT::TypeInfo::Create<int64_t>() };
                    }
                    //------------
                    if(std::type_index(typeid(float)) == type)
                    {
                        return BT::JsonExporter::Entry{ BT::Any(source.get<float>()), BT::TypeInfo::Create<float>() };
                    }
                    if(std::type_index(typeid(double)) == type)
                    {
                        return BT::JsonExporter::Entry{ BT::Any(source.get<double>()), BT::TypeInfo::Create<double>() };
                    }
                }

                else if(BT::demangle(typeid(bool)) == BT::demangle(type) && source.is_number_integer())
                {
                    return BT::JsonExporter::Entry{ BT::Any(BT::convertFromString<bool>(source.dump())), BT::TypeInfo::Create<bool>() };
                }
            }   
        }

        return expected_entry;
    }

    Expected<std::string> getEntryAsString(const std::string& key, const BT::Blackboard::Ptr blackboard, const bool lossy_json_compress_output)
    {
        // TODO Devis
        // std::pair<const Entry*, const Blackboard*> entry_bb_pair = blackboard->getEntry(key);
        // const Entry* val = entry_bb_pair.first;
        // const Blackboard* bb_ptr = entry_bb_pair.second;

        std::shared_ptr<BT::Blackboard::Entry> val = blackboard->getEntry(key);
        if (val /*&& bb_ptr*/)
        {
            // std::cout << "getEntryAsString: Extracting " << key << "\n" << 
            //                 "type=" << BT::demangle(val->info.type()) << " - " << val->info.typeName() << "\n"
            //                 "value=" << val->info.converter
            if(val->value.empty()) 
                return nonstd::make_unexpected(StrCat("Value for key [",key,"] empty"));

            else if(val->value.isString())
                //TODO Devis // return bb_ptr->replaceKeysWithStringValues(val->value.cast<std::string>());
                return val->value.cast<std::string>();
            
            // if(!val->info.isStronglyTyped()) 
            // {
            //     // if(val->value.isNumber())
            //     {
            //         // treat special and known cases
            //         if(val->value.type() == typeid(int64_t))
            //             return std::to_string(val->value.cast<int64_t>());
            //         else if(val->value.type() == typeid(uint64_t))
            //             return std::to_string(val->value.cast<uint64_t>());
            //         else if(val->value.type() == typeid(double))
            //             return std::to_string(val->value.cast<double>());

            //         else if(val->value.isType<nlohmann::json>()) // treat other special known and useful case
            //         {
            //             nlohmann::json json = val->value.cast<nlohmann::json>();
            //             return lossy_json_compress_output? lossyJsonCompress(json).dump() :  json.dump();
            //         }
                
            //     }
                
            //     try
            //     {
            //         // everything else just to be considered as something to be stringified through the json exporter
            //         BT::Expected<nlohmann::json> json = eutToJson(val->value, val->value.type());
            //         if(json.has_value())
            //             return lossy_json_compress_output? lossyJsonCompress(json.value()).dump() : json.value().dump();
            //     }
            //     catch(const std::runtime_error& re)
            //     {
            //         return nonstd::make_unexpected(StrCat("Missing type info for key [",key,
            //         "] and does not appear to be a primitive known type ",
            //         "(number, string, json) ",
            //         "that can be automatically infer to string. Errors details: ", re.what()));
            //     }
            // }

            else
            {
                // const auto to_str_converter_it = string_converters.find(val->info.typeName());
                // if(to_str_converter_it != string_converters.end())
                // TODO Devis
                // return bb_ptr->replaceKeysWithStringValues(val->port_info.toString(val->value));
                BT::Expected<nlohmann::json> json = eutToJson(val->value, val->info.type());
                if(json.has_value())
                    return lossy_json_compress_output? lossyJsonCompress(json.value()).dump() : json.value().dump();
                else
                    return nonstd::make_unexpected(StrCat("Type info for key [",key,
                        "] is " , val->info.typeName(),
                        " and cannot be automatically infer to string because of missing BT_JSON_CONVERTER() method specialization"));
            }
        }

        return nonstd::make_unexpected(StrCat("Entry for key [",key,"] missing"));
    }


    PortsValueMap getPortValuesMap(const BT::TreeNode& node, const PortDirection& dir, const bool lossy_json_compress_output)
    {
        std::unordered_map<std::string, std::string> ret;
        if(dir == PortDirection::INOUT)
            return ret;// not unsupported (actually you can just retrieve from either input or output and you will find it in both -> better retrieve it always from output considering the logic below) 

        PortsRemapping ports = dir == PortDirection::INPUT? node.config().input_ports : node.config().output_ports;

        if(ports.size()>0) 
        {
            for(auto it=ports.begin();it!=ports.end();it++)
            {
                bool isbbptr = TreeNode::isBlackboardPointer(it->second);
                if(isbbptr || dir == PortDirection::OUTPUT)//port value remapped -> needs to be retrieved from blackboard as string (toStr needs to be implemented for custom types), if not there catched error below
                {
                auto key = it->second;
                if(node.config().blackboard)
                {
                    if(isbbptr) key = static_cast<std::string>(TreeNode::stripBlackboardPointer(key));
                
                //TODO Devis
                // key = (node.config().blackboard->replaceKeysWithStringValues(key)).value_or(key);
                
                // Fancy json subfield access
                //   const auto json_path_opt = extractJsonPointer(key);
                //   std::string json_path;
                //   if(json_path_opt.has_value())
                //   {
                //     // It is a json pointer -> access the json-> then the json subfield as a string and insert that into the map
                //     key = extractJsonKey(key).value();
                //     const Any* any_value = (this->config().blackboard->getAny((key))).first;
                //     nlohmann::json destination;
                //     if(any_value->getJsonSubfield<nlohmann::json>(json_path_opt.value(), destination))
                //       ret.insert({it->first, BT::toStr<nlohmann::json>(destination)}); // toStr to guarantee that same treatment is guaranteed
                //   }
                //   else
                    {
                    // We don't have any particular json subptr, straightforward access
                    const auto opt_value = getEntryAsString(key, node.config().blackboard, lossy_json_compress_output);//(node.config().blackboard->getAsString((key)));
                    // std::cout << std::to_string(opt_value.has_value()) << "Got As String [" << key << "]: " << opt_value.value_or("") << "\n";
                    if(opt_value) ret.insert({it->first, opt_value.value()});
                    }

                }
                }
                else
                {
                ret.insert({it->first, it->second});
                }
            }
        }

        return ret;
    }


    Expected<nlohmann::json> getPortValueAsJson(const BT::TreeNode& node, const std::string& key, const BT::PortDirection dir)
    {
        std::string port_value_str;
        Any any_return;
        
        if(!node.config().manifest)
        {
            return nonstd::make_unexpected(StrCat("getInputAsJson() of node '", node.fullPath(),
                                                "' failed because the manifest is "
                                                "nullptr (WTF?) and the key: [",
                                                key, "] is missing"));
        }

        // maybe it is declared with a default value in the manifest
        auto port_manifest_it = node.config().manifest->ports.find(key);
        if(port_manifest_it == node.config().manifest->ports.end())
        {
            return nonstd::make_unexpected(StrCat("getInputAsJson() of node '", node.fullPath(),
                                                "' failed because the manifest doesn't "
                                                "contain the key: [",
                                                key, "]"));
        }
        const auto& port_info = port_manifest_it->second;

        // std::cout << "port_info.type " << BT::demangle(port_info.type()) << "\t typeName" <<  port_info.typeName() << "\n" << std::flush;

        const BT::PortsRemapping& ports = dir == PortDirection::OUTPUT? node.config().output_ports : node.config().input_ports;
        //TODO Extend to input and output
        auto input_port_it = ports.find(key);
        if(input_port_it != ports.end())
        {
            port_value_str = input_port_it->second;
        }
        else
        {
            // there is a default value
            if(port_info.defaultValue().empty())
            {
                return nonstd::make_unexpected(StrCat("getInputAsJson() of node '", node.fullPath(),
                                                    "' failed because nor the manifest or the "
                                                    "XML contain the key: [",
                                                    key, "]"));
            }
            if(port_info.defaultValue().isString())
            {
                any_return = BT::Any{port_info.defaultValue().cast<std::string>()};
            }
            else
            {
                any_return = (port_info.defaultValue());
            }
        }

        auto blackboard_ptr = BT::TreeNode::getRemappedKey(key, port_value_str);
        try
        {
            // pure string, not a blackboard key
            if(!blackboard_ptr)
            {
                try
                {
                    if(port_info.isStronglyTyped())
                        any_return = port_info.converter()(port_value_str);
                    else
                        any_return = BT::Any(port_value_str);
                }
                catch(std::exception& ex)
                {
                    return nonstd::make_unexpected(StrCat("getInputAsJson(): ", ex.what()));
                }
            }
            else
            {
                const auto& blackboard_key = blackboard_ptr.value();

                if(!node.config().blackboard)
                {
                    return nonstd::make_unexpected("getInputAsJson(): trying to access "
                                                "an invalid Blackboard");
                }

                // std::cout << "Remapped to key " << std::string(blackboard_key) << "\n" << std::flush;
                if(auto any_ptr = node.config().blackboard->getAnyLocked(std::string(blackboard_key)))
                    any_ptr->copyInto(any_return);

                else
                    return nonstd::make_unexpected(StrCat("getInputAsJson() failed because it was unable to "
                                                    "find the key [",
                                                    key, "] remapped to [", blackboard_key, "]"));
            }
        }
        catch(std::exception& err)
        {
            return nonstd::make_unexpected(err.what());
        }

        return eutToJson(any_return, port_info.type());
    }
}
};