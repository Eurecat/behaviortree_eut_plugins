#ifndef DEBUG_UTIL_H
#define DEBUG_UTIL_H

#include "behaviortree_cpp/basic_types.h"
#include "behaviortree_eut_plugins/eut_basic_types.h"
#include "behaviortree_eut_plugins/flatbuffers/bt_flatbuffer_helper_eut.h"

namespace BT
{

    //cmds sent by Groot or other client in debugging phase
    enum class DebugCmd
    {
        // NO PAYLOAD EXPECTED
        RESUME = 0,
        // TICK_ROOT = 1, // might be rethought in the future, for now disabled
        TICK_STEP = 2,
        PAUSE = 3,

        // PAYLOAD EXPECTED
        SETBP = 4, // payload: UID
        RMPB = 5,  // payload: UID
        SETPORT = 6,// payload: UID, port_name, port_value

        SETDISABLED = 7,// payload: UID
        SETENABLED = 8,// payload: UID

        OVERRIDE_TICK = 9,// payload: UID, status, skip_tick

        BAD_REQUEST
    };

    struct DebugPayload
    {
        int uid;
        
        DebugPayload(const int id) :
            uid(id) 
            { };

        virtual ~DebugPayload(){};
    };

    struct PortDebugPayload : DebugPayload
    {
        std::string port_name;
        std::string port_value;
        
        PortDebugPayload(const int uid, const std::string& name, const std::string& value) : DebugPayload(uid),
            port_name(name), port_value(value) 
            { };
            
        
        ~PortDebugPayload()override{};
    };

    struct TickDebugPayload : DebugPayload
    {
        NodeAdvancedStatus status;
        bool skip_tick;

        TickDebugPayload(const int uid, const NodeAdvancedStatus s, const bool skip) : DebugPayload(uid),
            status(s), skip_tick(skip) 
            { };

        ~TickDebugPayload()override{};
    };

    inline bool payloadRequired(const DebugCmd& cmd)
    {
        return cmd > DebugCmd::PAUSE;
    }

    template <>
    [[nodiscard]] DebugCmd convertFromString<BT::DebugCmd>(StringView str);

    template <>
    [[nodiscard]] std::string toStr<BT::DebugCmd>(const BT::DebugCmd& cmd);

    template <>
    [[nodiscard]] std::string toStr<BT::DebugPayload>(const BT::DebugPayload& body);

    template <>
    [[nodiscard]] std::string toStr<BT::PortDebugPayload>(const BT::PortDebugPayload& body);

    template <>
    [[nodiscard]] std::string toStr<BT::TickDebugPayload>(const BT::TickDebugPayload& body);
    

    using DebugStatusChangeSignal = Signal<TimePoint, const TreeNode&, NodeAdvancedStatus, NodeAdvancedStatus>;
    using DebugStatusChangeSubscriber = DebugStatusChangeSignal::Subscriber;
    using DebugStatusChangeCallback = DebugStatusChangeSignal::CallableFunction;

    enum class Breakpoint
    {
        NO_BP = 0,
        BP = 1,
        OBP = 2 // one usage breakpoint
    };

    enum class Override
    {
        UNDEFINED = 0,
        TICK = 1,
        NOTICK = 2
    };

    class DebuggableTree;

    class DebugTreeNode
    {
        public:
            typedef std::shared_ptr<DebugTreeNode> Ptr;
            
            DebugTreeNode(const BT::TreeNode::Ptr node, DebuggableTree* debug_tree = nullptr): 
                node_(node), 
                debug_tree_(debug_tree), 
                dstatus_(toNodeAdvancedStatus(node->status())),
                override_(Override::UNDEFINED),
                override_status_(BT::NodeStatus::IDLE),
                breakpoint_(Breakpoint::NO_BP)
            {
                addDefaultPreTickFunction();
                addDefaultPostTickFunction();
            }

            ~DebugTreeNode() = default;

            void addBreakpoint(bool one_time = false);
            void rmBreakpoint();
            bool hasBreakpoint()const { return breakpoint_ == Breakpoint::BP;}
            bool isPaused()const;

            void setDisabled(const bool disabled);

            void setOverrideTick(const BT::NodeAdvancedStatus override_status, const bool skip_tick = true);

            bool updPortValue(const std::string& port_name, const std::string& port_value);
            

            DebugStatusChangeSubscriber subscribeToDebugStatusChange(DebugStatusChangeCallback callback)
            {
                return dstate_change_signal_.subscribe(std::move(callback));
            }

            void setAdvancedStatus(NodeAdvancedStatus advanced_status);

            BT::TreeNode::Ptr node(){return node_;};

            void addDefaultPreTickFunction();
            void addDefaultPostTickFunction();

        protected:
                struct TreeNodeHelper : TreeNode 
                {
                    // Bring protected methods into protected scope for this class
                    using TreeNode::config; 
                    using TreeNode::preConditionsScripts;
                };

                virtual BT::NodeStatus checkOverridePreTick();

                BT::TreeNode::Ptr node_;
                DebuggableTree* debug_tree_;
                NodeAdvancedStatus dstatus_;
                std::condition_variable dstate_condition_variable_;
                mutable std::mutex dstate_mutex_;
                DebugStatusChangeSignal dstate_change_signal_;
                std::atomic_bool disabled_;
                Override override_;
                BT::NodeStatus override_status_;
                BT::ScriptFunction src_skip_if_value_;
        private:
            std::atomic<Breakpoint> breakpoint_;
    };

    class DebuggableTree
    {


        public:
            enum class ExecMode
            {
                // 
                DEFAULT = 0,
                DEBUG_BP = 1,
                DEBUG_TICK = 2,
                DEBUG_STEP = 3,
                STOP = 4
            };
            
            typedef std::shared_ptr<DebuggableTree> Ptr;

            DebuggableTree(std::shared_ptr<Tree> tree, bool pausable = true, bool start_paused = true)
                : tree_(tree), exec_mode_(ExecMode::DEFAULT)
            {
                if(pausable)
                {
                    for(auto subtree : tree_->subtrees)
                        for(auto node : subtree->nodes)
                            debug_wrappers_.emplace(node->UID(), std::make_shared<BT::DebugTreeNode>(node, this));
                    
                    if(start_paused) setBreakpoint(tree_->rootNode()->UID(), true);
                    exec_mode_ = start_paused? ExecMode::DEBUG_STEP : ExecMode::DEBUG_BP;
                }

                tree_->initialize();
            }


            std::shared_ptr<Tree> tree(){return tree_;};
            const Tree& treeRef()const {return *tree_;};

            bool inDebugMode() const {return exec_mode_ != ExecMode::DEFAULT && exec_mode_ != ExecMode::STOP;}

            bool inDebugTickRootMode() const {return exec_mode_ == ExecMode::DEBUG_TICK;}

            bool inDebugTickStepMode() const {return exec_mode_ == ExecMode::DEBUG_STEP;}

            bool isPaused();

            bool hasBeenStopped() const {return exec_mode_ == ExecMode::STOP;}

            bool inTree(const BT::TreeNode* node);

            std::set<uint16_t> bpNodesUID()const;

            unsigned long int nTicked()const {return ticked_;}
            
            void updateTreeNodeStatus(BT::TreeNode* node, NodeStatusVariant status) 
            {
                struct TreeNodeHelper : TreeNode {
                    using TreeNode::setStatus; // Bring protectedMethod into public scope
                };

                std::visit([node, this](auto&& arg) {
                    using T = std::decay_t<decltype(arg)>;
                    if constexpr (std::is_same_v<T, NodeStatus>) 
                    {
                       static_cast<TreeNodeHelper&>(*(node)).setStatus(arg);
                    } 
                    else if constexpr (std::is_same_v<T, NodeAdvancedStatus>) 
                    {
                        auto debug_wrap_it = debug_wrappers_.find(node->UID());
                        if(debug_wrap_it != debug_wrappers_.end())
                        {
                            debug_wrap_it->second->setAdvancedStatus(arg);
                        }
                    }
                }, status);
            }

            /*
                Put current execution in pause
            */
            void debugPause(BT::TreeNode* node);

            /*
                Resume paused execution, tick1 equals to true if we want to move forward just one step
            */
            bool debugResume(const ExecMode resume_mode = ExecMode::DEBUG_BP);

            /*
                Set the breakpoint from node with given uid,
                return true if found, false otherwise
            */
            bool setBreakpoint(const uint16_t uid, const bool one_time = false);

            /*
                Remove the breakpoint from node with given uid,
                return true if found, false otherwise
            */
            bool rmBreakpoint(const uint16_t uid);

            /*
                Set tick override command to node with given uid
                return true if found and modification is performed, false otherwise

                Do we need to constrain the update to a paused node? I don't think so 
            */
            bool setOverrideTick(const std::shared_ptr<DebugPayload> debugPayload);

            /*
                Set node with given uid to be skipped or not, overriding its current _skip_if expression
                return true if found, false otherwise
            */
            bool setDisabled(const uint16_t uid, const bool disabled);

            /*
                Update port value if specified port_name is present among nodes with specified node_uid
            */
            bool updPortValue(const std::shared_ptr<DebugPayload> body);

            std::vector<DebugStatusChangeSubscriber> instantiateDebugSubscribers(DebugStatusChangeCallback callback)
            {
                std::vector<DebugStatusChangeSubscriber> subs(debug_wrappers_.size());
                uint16_t i = 0;
                for(const auto & [ uid, dnode ] : debug_wrappers_)
                {
                    subs[i++]=dnode->subscribeToDebugStatusChange(callback);
                }
                return subs;
            }

            const std::unordered_map<uint16_t, BT::DebugTreeNode::Ptr>& debugWrappers() const
            {
                return debug_wrappers_;
            }

            
        protected:
            std::shared_ptr<Tree> tree_;
            std::unordered_map<uint16_t, BT::DebugTreeNode::Ptr> debug_wrappers_;
            
            ExecMode exec_mode_;
            bool paused_{false};
            unsigned long int ticked_{0};
            std::condition_variable debug_cv_;
            std::mutex debug_mtx_;
    };

// PortsValueMap getBlackboardValuesMap(const BT::TreeNode& node);
};

#endif