#ifndef EUT_BASIC_TYPES_H
#define EUT_BASIC_TYPES_H

#include "behaviortree_cpp/basic_types.h"
#include "behaviortree_cpp/contrib/json.hpp"

namespace BT
{
    using ToStringConverter = std::function<std::string(const BT::Any&)>;

    using ToStringConvertersMap = std::unordered_map<std::string, ToStringConverter>;

    template <>
    [[nodiscard]] std::string toStr<BT::Any>(const BT::Any& value);

    // template <> 
    //     [[nodiscard]] inline std::string toStr(const bool& value){return std::to_string(value);}
    template <> 
        [[nodiscard]] inline std::string toStr(const int& value){return std::to_string(value);}
    template <> 
        [[nodiscard]] inline std::string toStr(const long& value){return std::to_string(value);}
    template <> 
        [[nodiscard]] inline std::string toStr(const unsigned short& value){return std::to_string(value);}
    template <> 
        [[nodiscard]] inline std::string toStr(const short& value){return std::to_string(value);}
    template <> 
        [[nodiscard]] inline std::string toStr(const long long& value){return std::to_string(value);}
    template <> 
        [[nodiscard]] inline std::string toStr(const unsigned& value){return std::to_string(value);}
    template <> 
        [[nodiscard]] inline std::string toStr(const unsigned long& value){return std::to_string(value);}
    template <> 
        [[nodiscard]] inline std::string toStr(const unsigned long long& value){return std::to_string(value);}
    template <> 
        [[nodiscard]] inline std::string toStr(const float& value){return std::to_string(value);}
    template <> 
        [[nodiscard]] inline std::string toStr(const double& value){return std::to_string(value);}
    template <> 
        [[nodiscard]] inline std::string toStr(const long double& value){return std::to_string(value);}
    // template <> 
    //     [[nodiscard]] inline std::string toStr(const std::string& value){return (value);}

    template <> 
        [[nodiscard]] std::string toStr<unsigned char>(const unsigned char& value);
    template <> 
        [[nodiscard]] std::string toStr<char>(const char& value);

    template <> 
        [[nodiscard]] std::string toStr<std::vector<int>>(const std::vector<int>& value);
    template <> 
        [[nodiscard]] std::string toStr<std::vector<double>>(const std::vector<double>& value);
    template <> 
        [[nodiscard]] std::string toStr<std::vector<std::string>>(const std::vector<std::string>& value);

    // template <>
    //     [[nodiscard]] std::string toStr<BT::NodeStatus>(const BT::NodeStatus& status);

    // template <>
    //     [[nodiscard]] std::string toStr<BT::PreCond>(const BT::PreCond& pre);

    template <>
        [[nodiscard]] std::string toStr<BT::Any>(const BT::Any& value);

    // /**
    //  * @brief toStr converts NodeStatus to string. Optionally colored.
    //  */
    // std::string toStr(BT::NodeStatus status, bool colored);

    template <>
        [[nodiscard]] std::string toStr<nlohmann::json>(const nlohmann::json& json);

    
    enum class NodeAdvancedStatus
    {  
        IDLE = 0,
        RUNNING = 1,
        SUCCESS = 2,
        FAILURE = 3,
        SKIPPED = 4,
        PAUSED = 5
    };

    template <>
        [[nodiscard]] NodeAdvancedStatus convertFromString<NodeAdvancedStatus>(StringView str);

    template <>
        [[nodiscard]] std::string toStr<NodeAdvancedStatus>(const NodeAdvancedStatus& value);

    static inline NodeStatus toNodeStatus(NodeAdvancedStatus advanced_status)
    {
        switch(advanced_status)
        {
            case NodeAdvancedStatus::IDLE: return NodeStatus::IDLE;
            case NodeAdvancedStatus::RUNNING: return NodeStatus::RUNNING;
            case NodeAdvancedStatus::SUCCESS: return NodeStatus::SUCCESS;
            case NodeAdvancedStatus::FAILURE: return NodeStatus::FAILURE;
            case NodeAdvancedStatus::SKIPPED: return NodeStatus::SKIPPED;
            default: return NodeStatus::IDLE;
        }

    };

    static inline NodeAdvancedStatus toNodeAdvancedStatus(NodeStatus status)
    {
        switch(status)
        {
            case NodeStatus::IDLE: return NodeAdvancedStatus::IDLE;
            case NodeStatus::RUNNING: return NodeAdvancedStatus::RUNNING;
            case NodeStatus::SUCCESS: return NodeAdvancedStatus::SUCCESS;
            case NodeStatus::FAILURE: return NodeAdvancedStatus::FAILURE;
            case NodeStatus::SKIPPED: return NodeAdvancedStatus::SKIPPED;
            default: return NodeAdvancedStatus::IDLE;
        }

    };

    using NodeStatusVariant = std::variant<NodeStatus, NodeAdvancedStatus>;
}

#endif