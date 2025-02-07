#include "behaviortree_eut_plugins/eut_debug.h"

#include "behaviortree_cpp/xml_parsing.h"

#include <boost/algorithm/string/replace.hpp>


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
                    
                    // avoid compilation warnings
                    (void)node;
                    (void)curr_status;

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
