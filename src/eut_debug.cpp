#include "behavior_tree_eut_plugins/eut_debug.h"

#include "behaviortree_cpp/json_export.h"
#include "behaviortree_cpp/xml_parsing.h"

#include <boost/algorithm/string/replace.hpp>

#define JSON_TRUNC_STR_DIM 16

namespace BT
{

    template <>
    DebugCmd convertFromString<DebugCmd>(StringView str)
    {
        auto std_str = std::stoi(std::string(str));
        if( std_str == 0 ) return DebugCmd::RESUME ;
        // if( std_str == 1 ) return DebugCmd::TICK_ROOT  ;
        if( std_str == 2 ) return DebugCmd::TICK_STEP  ;
        if( std_str == 3 ) return DebugCmd::PAUSE  ;
        if( std_str == 4 ) return DebugCmd::SETBP  ;
        if( std_str == 5 ) return DebugCmd::RMPB   ;
        if( std_str == 6 ) return DebugCmd::SETPORT;
        if( std_str == 7 ) return DebugCmd::SETDISABLED   ;
        if( std_str == 8 ) return DebugCmd::SETENABLED;
        if( std_str == 9 ) return DebugCmd::OVERRIDE_TICK;
        
        throw RuntimeError("convertFromString(): invalid DebugCmd conversion");
    }

    template <>
    std::string toStr<DebugCmd>(const DebugCmd& cmd)
    {
        switch(cmd)
        {
            case DebugCmd::RESUME: return "0";
            // case DebugCmd::TICK_ROOT: return "1";
            case DebugCmd::TICK_STEP: return "2";
            case DebugCmd::PAUSE: return "3";
            case DebugCmd::SETBP: return "4";
            case DebugCmd::RMPB: return "5";
            case DebugCmd::SETPORT: return "6";
            case DebugCmd::SETDISABLED: return "7";
            case DebugCmd::SETENABLED: return "8";
            case DebugCmd::OVERRIDE_TICK: return "9";
            default:
            return "";
        }
        return "";
    }

    template <>
    std::string toStr<DebugPayload>(const DebugPayload& body)
    {
        return "uid:"+std::to_string(body.uid);
    }

    template <>
    std::string toStr<PortDebugPayload>(const PortDebugPayload& body)
    {  
        const std::string prefix = toStr<DebugPayload>(body);
        std::string escaped_port_value = boost::replace_all_copy(body.port_value,  ",", "\\,");
        return prefix + ",port_name:" + body.port_name + ",port_value:" + escaped_port_value;
    }

    template <>
    std::string toStr<TickDebugPayload>(const TickDebugPayload& body)
    {
        const std::string prefix = toStr<DebugPayload>(body);
        return prefix + ",status:" + toStr<NodeAdvancedStatus>(body.status) + ",skip_tick:" + toStr(body.skip_tick);
    }

    bool missingTypeInfo(const std::type_index& type)
    {
        return type == typeid(BT::Any) || type == typeid(void);
    }

    static
    void fixTypeMismatchJson(nlohmann::json& json, const BT::PortInfo& info)
    {
        if(info.type() == typeid(bool))
            json = BT::convertFromString<bool>(json.dump());
    }

    nlohmann::json cleanJson(nlohmann::json& json)
    {
        if (json.is_object()) 
        {
            for (auto it = json.begin(); it != json.end(); ++it) 
            {
                cleanJson(it.value());
            }
        } 
        else if (json.is_array()) 
        {
            for (auto& element : json) 
            {
                cleanJson(element);
            }
        } 
        else if (json.is_string()) 
        {
            std::string str = json.get<std::string>();
            if (str.size() > JSON_TRUNC_STR_DIM) 
            {
                str = str.substr(0, 16); // Truncate to 16 characters
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

    Expected<std::string> getEntryAsString(const std::string& key, const BT::Blackboard::Ptr blackboard, const bool clean_json_output)
    {
        // TODO Devis
        // std::pair<const Entry*, const Blackboard*> entry_bb_pair = blackboard->getEntry(key);
        // const Entry* val = entry_bb_pair.first;
        // const Blackboard* bb_ptr = entry_bb_pair.second;
        std::shared_ptr<BT::Blackboard::Entry> val = blackboard->getEntry(key);
        if (val /*&& bb_ptr*/)
        {
            if(val->value.empty()) 
                return nonstd::make_unexpected(StrCat("Value for key [",key,"] empty"));

            else if(val->value.isString())
                //TODO Devis // return bb_ptr->replaceKeysWithStringValues(val->value.cast<std::string>());
                return val->value.cast<std::string>();
            
            else if(missingTypeInfo(val->info.type())) 
            {
                // if(val->value.isNumber())
                //     return val->value.cast<std::string>();
                
                // else if(val->value.isJson())
                //     return bb_ptr->replaceKeysWithStringValues(BT::toStr<nlohmann::json>(val->value.cast<nlohmann::json>()));
                
                try
                {
                    // std::cout << "non so il tipooooo di " << key << "(" << BT::demangle(val->info.type()) << ")\n" << std::flush;
                    return val->value.cast<std::string>();
                }
                catch(const std::runtime_error& re)
                {
                    return nonstd::make_unexpected(StrCat("Missing type info for key [",key,
                    "] and does not appear to be a primitive known type ",
                    "(number, string, json) ",
                    "that can be automatically infer to string. Errors details: ", re.what()));
                }
            }

            else
            {
                // const auto to_str_converter_it = string_converters.find(val->info.typeName());
                // if(to_str_converter_it != string_converters.end())
                // TODO Devis
                // return bb_ptr->replaceKeysWithStringValues(val->port_info.toString(val->value));
                nlohmann::json json;
                if(val->info.type() == typeid(nlohmann::json)) json = val->value.tryCast<nlohmann::json>().value_or(nlohmann::json{});
                if(!json.empty() || JsonExporter::get().toJson(val->value, json))
                    return clean_json_output? cleanJson(json).dump() : json.dump();
                else
                    return nonstd::make_unexpected(StrCat("Type info for key [",key,
                        "] is " , val->info.typeName(),
                        " and cannot be automatically infer to string because of missing BT_JSON_CONVERTER() method specialization"));
            }
        }

        return nonstd::make_unexpected(StrCat("Entry for key [",key,"] missing"));
    }


    PortsValueMap getPortValuesMap(const BT::TreeNode& node, const PortDirection& dir, const bool clean_json_output)
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
                    const auto opt_value = getEntryAsString(key, node.config().blackboard, clean_json_output);//(node.config().blackboard->getAsString((key)));
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
                    any_return = port_info.converter()(port_value_str);
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

        nlohmann::json json;
        // std::cout << "Any_return castedType = " << BT::demangle(any_return.castedType()) << 
        //     " \t type = " << BT::demangle(any_return.type()) << "\n" << std::flush;
        JsonExporter::get().toJson(any_return, json);
        fixTypeMismatchJson(json, port_info);
        return json;
    }

    // DebuggableTree
        
    bool DebuggableTree::isPaused() 
    {
        std::unique_lock<std::mutex> lock(debug_mtx_); 
        return inDebugMode() && paused_;
    }

    bool DebuggableTree::inTree(const BT::TreeNode* node)
    {
        for(auto const& s : tree_->subtrees)
        {
            for(auto const& n : s->nodes)
            {
                if(n->UID() == node->UID())
                    return true;
            }
        }
        return false;
    }

    /*
        Put current execution in pause
    */
    void DebuggableTree::debugPause(BT::TreeNode* node)
    {
        if(node == nullptr || !inTree(node))
        return;

        if(exec_mode_ != ExecMode::DEFAULT)
        {
            auto prev_status = node->status();
            updateTreeNodeStatus(node, NodeAdvancedStatus::PAUSED);
            std::unique_lock<std::mutex> lck(debug_mtx_);
            paused_ = true;
            debug_cv_.wait(lck);
            paused_ = false;
            updateTreeNodeStatus(node, toNodeAdvancedStatus(prev_status));
        }
        else 
        throw RuntimeError("Calling debugPause(), but you're not in DEBUG mode");
    }

    /*
        Resume paused execution, tick1 equals to true if we want to move forward just one step
    */
    bool DebuggableTree::debugResume(const ExecMode resume_mode)
    {
        if(exec_mode_ != ExecMode::DEFAULT)
        {
            std::unique_lock<std::mutex> lck(debug_mtx_);
            exec_mode_ = resume_mode; // mark it in the exec mode so we can pause at the next tick
            debug_cv_.notify_all();
            return true;
        }
        else 
            return false;//throw RuntimeError("Calling debugResume(), but you're not in DEBUG mode");
    }

    bool DebugTreeNode::isPaused() const
    {
        std::unique_lock<std::mutex> UniqueLock(dstate_mutex_);
        return dstatus_ == NodeAdvancedStatus::PAUSED;
    }

    void DebugTreeNode::setAdvancedStatus(NodeAdvancedStatus advanced_status)
    {
        NodeAdvancedStatus prev_dstatus;
        {
            std::unique_lock<std::mutex> UniqueLock(dstate_mutex_);
            prev_dstatus = dstatus_;
            dstatus_ = advanced_status;
            
            // std::cout << "CHANGING ADV STATUS OF NODE " << node()->UID() << 
            //     " from " << toStr(prev_dstatus) <<
            //     " to " << toStr(advanced_status) << std::endl;

            if(dstatus_ == BT::NodeAdvancedStatus::PAUSED && breakpoint_ == Breakpoint::OBP)
                rmBreakpoint();
        }
        if(prev_dstatus != dstatus_)
        {
            dstate_condition_variable_.notify_all();
            dstate_change_signal_.notify(std::chrono::high_resolution_clock::now(), *node_,
                                        prev_dstatus, dstatus_);
        }
    }


    BT::NodeStatus DebugTreeNode::checkOverridePreTick()
    {
        if(override_ == Override::NOTICK && isStatusCompleted(override_status_))// override status and do not do tick
        {
            const auto override_status = override_status_;
            override_status_ = BT::NodeStatus::IDLE;
            override_ = Override::UNDEFINED;
            return override_status;
        }
        return BT::NodeStatus::SKIPPED;
    }


    void DebugTreeNode::addBreakpoint(const bool one_time)
    {
        if(!debug_tree_) return;

        node()->setPreTickFunction(
                [&](TreeNode& node) -> NodeStatus 
                    {
                        debug_tree_->debugPause(&node);
                        return checkOverridePreTick();
                    });
        breakpoint_ = one_time? Breakpoint::OBP : Breakpoint::BP;
    }

    void DebugTreeNode::addDefaultPreTickFunction()
    {
        node()->setPreTickFunction(
            [&](TreeNode& node) -> NodeStatus 
                {
                    if(debug_tree_->inDebugTickStepMode())
                        debug_tree_->debugPause(&node);
                    return checkOverridePreTick();
                });
    }

    void DebugTreeNode::addDefaultPostTickFunction()
    {
        node()->setPostTickFunction(
            [&](TreeNode& node, NodeStatus curr_status) -> NodeStatus 
                {   
                    // without -Wunused-parameter this lambda will return warning for unused parameters, but that's completely fine

                    if(override_ == Override::TICK && isStatusCompleted(override_status_))// override status and do not do tick
                    {
                        const auto override_status = override_status_;
                        override_status_ = BT::NodeStatus::IDLE;
                        override_ = Override::UNDEFINED;
                        return override_status;
                    }
                    return NodeStatus::SKIPPED;
                });
    }

    void DebugTreeNode::rmBreakpoint()
    {
        addDefaultPreTickFunction();
        breakpoint_ = Breakpoint::NO_BP;
    }

    
    void DebugTreeNode::setDisabled(const bool disabled)
    {
        std::array<BT::ScriptFunction, size_t(PreCond::COUNT_)>& executors = static_cast<TreeNodeHelper&>(*(node_)).preConditionsScripts();
        if(disabled) 
        {
            src_skip_if_value_ = executors[size_t(PreCond::SKIP_IF)];
            executors[size_t(PreCond::SKIP_IF)] = ParseScript("1").value();
        }
        else
            executors[size_t(PreCond::SKIP_IF)] = src_skip_if_value_;
    }

    bool DebugTreeNode::updPortValue(const std::string& port_name, const std::string& port_value)
    {
        BT::NodeConfig& config = static_cast<TreeNodeHelper&>(*(node_)).config();
        const auto& port_it = config.input_ports.find(port_name);
        if(port_it != config.input_ports.end())
        {
            port_it->second = port_value;
            return true;
        }
        return false;
    }


    void DebugTreeNode::setOverrideTick(const BT::NodeAdvancedStatus override_status, const bool skip_tick)
    {
        if(isStatusCompleted(toNodeStatus(override_status)))
        {
            override_status_ = toNodeStatus(override_status);
            override_ = skip_tick? Override::NOTICK : Override::TICK;
        }
    }

    /*
        Set the breakpoint from node with given uid,
        return true if found, false otherwise
    */
    bool DebuggableTree::setBreakpoint(const uint16_t uid, const bool one_time)
    {
        if(!tree_) return false;
        
        auto debug_wrap_it = debug_wrappers_.find(uid);
        if(debug_wrap_it != debug_wrappers_.end())
        {
            debug_wrap_it->second->addBreakpoint(one_time);
            return true;
        }
        return false;
    }

    /*
        Remove the breakpoint from node with given uid,
        return true if found, false otherwise
    */
    bool DebuggableTree::rmBreakpoint(const uint16_t uid)
    {
        if(!tree_) return false;

        auto debug_wrap_it = debug_wrappers_.find(uid);
        if(debug_wrap_it != debug_wrappers_.end())
        {
            debug_wrap_it->second->rmBreakpoint();
            return true;
        }
        return false;
    }


    bool DebuggableTree::setDisabled(const uint16_t uid, const bool disabled)
    {
        if(!tree_) return false;

        auto debug_wrap_it = debug_wrappers_.find(uid);
        if(debug_wrap_it != debug_wrappers_.end())
        {
            debug_wrap_it->second->setDisabled(disabled);
            return true;
        }
        return false;
    }

    bool DebuggableTree::updPortValue(const std::shared_ptr<DebugPayload> body)
    {
        if(const PortDebugPayload* port_debug_payload = dynamic_cast<const PortDebugPayload*>(body.get()))
        {
            auto debug_node_it = debug_wrappers_.find(port_debug_payload->uid);
            if(debug_node_it != debug_wrappers_.end())
            {
                return debug_node_it->second->updPortValue(port_debug_payload->port_name, port_debug_payload->port_value);
            }
        }
        return false;
    }

    std::set<uint16_t> DebuggableTree::bpNodesUID() const
    {
        std::set<uint16_t> res;
        for(const auto& debug_node : debug_wrappers_)
            if(debug_node.second->hasBreakpoint()) res.insert(debug_node.second->node()->UID());
        return res;
    }

    bool DebuggableTree::setOverrideTick(const std::shared_ptr<DebugPayload> debugPayload)
    {
        if(!tree_) return false;

        if(const TickDebugPayload* tick_debug_payload = dynamic_cast<const TickDebugPayload*>(debugPayload.get()))
        {
            auto debug_wrap_it = debug_wrappers_.find(tick_debug_payload->uid);
            if(debug_wrap_it != debug_wrappers_.end())
            {
                debug_wrap_it->second->setOverrideTick(tick_debug_payload->status, tick_debug_payload->skip_tick);
                return true;
            }
        }

        return false;
    }

}  // namespace BT
