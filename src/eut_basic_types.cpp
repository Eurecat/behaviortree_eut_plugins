#include "behaviortree_eut_plugins/eut_basic_types.h"

namespace BT
{
    template <>
    std::string toStr<Any>(const Any& value)
    {
        return value.empty()? "" : value.cast<std::string>();
    }

    template <> 
    std::string toStr<unsigned char>(const unsigned char& value)
    {
        // (NOTE if you lose the template signature declaration in the .h for this template specialization, you get a "stack smashing detected" error)
        // return std::string{1, *(reinterpret_cast<char*>(&value))};// solution below should be better/safer
        std::stringstream ss;
        ss << value;
        std::string res = ss.str();
        return res;
    }

    template <> 
    std::string toStr<char>(const char& value)
    {
        return std::string{1, value};
    }


    template <>
    std::string toStr<nlohmann::json>(const nlohmann::json& json)
    {
        return json.dump();
    }

    template <> 
    std::string toStr<std::vector<int>>(const std::vector<int>& value)
    {
        std::string res = "";
        for(const auto& item : value)
            res += BT::toStr(item) + ";";
        return res;
    }

    template <> 
    std::string toStr<std::vector<double>>(const std::vector<double>& value)
    {
        std::string res = "";
        for(const auto& item : value)
            res += BT::toStr(item) + ";";
        return res;
    }

    template <> 
    std::string toStr<std::vector<std::string>>(const std::vector<std::string>& value)
    {
        std::string res = "";
        for(const auto& item : value)
            res += BT::toStr(item) + ";";
        return res;
    }

    template <>
    NodeAdvancedStatus convertFromString<NodeAdvancedStatus>(StringView str)
        {
        if(str == "IDLE")
            return NodeAdvancedStatus::IDLE;
        if(str == "RUNNING")
            return NodeAdvancedStatus::RUNNING;
        if(str == "SUCCESS")
            return NodeAdvancedStatus::SUCCESS;
        if(str == "FAILURE")
            return NodeAdvancedStatus::FAILURE;
        if(str == "SKIPPED")
            return NodeAdvancedStatus::SKIPPED;
        if(str == "PAUSED")
            return NodeAdvancedStatus::PAUSED;

        throw RuntimeError(std::string("Cannot convert this to NodeAdvancedStatus: ") +
                            static_cast<std::string>(str));
    }

    template <>
    std::string toStr<NodeAdvancedStatus>(const NodeAdvancedStatus& status)
    {
        switch(status)
        {
            case NodeAdvancedStatus::SUCCESS:
                return "SUCCESS";
            case NodeAdvancedStatus::FAILURE:
                return "FAILURE";
            case NodeAdvancedStatus::RUNNING:
                return "RUNNING";
            case NodeAdvancedStatus::IDLE:
                return "IDLE";
            case NodeAdvancedStatus::SKIPPED:
                return "SKIPPED";
            case NodeAdvancedStatus::PAUSED:
                return "PAUSED";
        }
        return "";
    }

    template <> 
        nlohmann::json convertFromString(StringView str)
    {
        if(str.empty())
            return nlohmann::json{};
        else 
        {
            try
            {
                return nlohmann::json::parse(str.data());
            }
            catch (const nlohmann::json::parse_error &e)
            {
                throw BT::RuntimeError("Failed to parse JSON: ", e.what());
            }
        }
    }
};