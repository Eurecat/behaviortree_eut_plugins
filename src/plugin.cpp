#include "behaviortree_cpp/bt_factory.h"

#include "behaviortree_eut_plugins/eut_basic_types.h"
#include "behaviortree_cpp/contrib/json.hpp"


#include "behaviortree_eut_plugins/decorators/ForEachLoopNode.h"
#include "behaviortree_eut_plugins/conditions/CheckEmptyJsonNode.h"
#include "behaviortree_eut_plugins/conditions/CheckBoolNode.h"
#include "behaviortree_eut_plugins/conditions/ComparisonNode.h"
#include "behaviortree_eut_plugins/actions/RandomValueNode.h"
#include "behaviortree_eut_plugins/actions/RandomSequenceValueNode.h"
#include "behaviortree_eut_plugins/actions/AccessJsonFieldNode.h"
#include "behaviortree_eut_plugins/actions/SplitStringToJsonArray.h"
#include "behaviortree_eut_plugins/actions/AddArrayToNode.h"
#include "behaviortree_eut_plugins/actions/AddKeyValueToJson.h"
#include "behaviortree_eut_plugins/actions/ConvertJsonToNode.h"
#include "behaviortree_eut_plugins/actions/InitializeNode.h"
#include "behaviortree_eut_plugins/actions/GetSizeNode.h"
#include "behaviortree_eut_plugins/actions/ConcatenateStrings.h"
#include "behaviortree_eut_plugins/actions/SetBlackboardEutNode.h"

// A custom struct  that I want to visualize in Groot
struct Position2D
{
  double x;
  double y;
  
  Position2D()
        : x(0.0), y(0.0)
    {
    } 

  Position2D(double _x, double _y)
        : x(_x), y(_y)
    {
    }

  //Copy constructor
    Position2D(const Position2D& other)
        : x(other.x), y(other.y)
    {
    }
};

// This macro will generate the code that is needed to convert
// the object to/from JSON.
// You still need to call BT::RegisterJsonDefinition<Position2D>()
// in main()
BT_JSON_CONVERTER(Position2D, pos)
{
  add_field("x", &pos.x);
  add_field("y", &pos.y);
}

// Template specialization to converts a string to Position2D.
namespace BT
{
    template <> [[nodiscard]] Position2D convertFromString(StringView str)
    {
        // We expect real numbers separated by semicolons
        auto parts = splitString(str, ';');
        if (parts.size() != 2)
        {
            throw RuntimeError("invalid input)");
        }
        else
        {
            Position2D output{0.0,0.0};
            output.x     = convertFromString<double>(parts[0]);
            output.y     = convertFromString<double>(parts[1]);
            // std::cout << "Building a position 2d object " << output.x << "; " << output.y << "\n" << std::flush;
            return output;
        }
    }

    template <> [[nodiscard]] std::string toStr<Position2D>(const Position2D& value)
    {
       return std::to_string(value.x) + ";" + std::to_string(value.y);
    }
} // end namespace BT

// Simple Action that updates an instance of Position2D in the blackboard
class UpdatePosition : public BT::SyncActionNode
{
public:
  UpdatePosition(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config)
  {}

  BT::NodeStatus tick() override
  {
    const auto in_pos = getInput<Position2D>("pos_in");
    if(!in_pos.has_value())
      return BT::NodeStatus::FAILURE;
    Position2D _pos = in_pos.value();
    _pos.x += getInput<double>("x").value_or(0.0);
    _pos.y += getInput<double>("y").value_or(0.0);
    std::cout << "Updating pos to " << std::to_string(_pos.x) << ", " << std::to_string(_pos.y) << std::endl;
    setOutput("pos_out", _pos);
    return BT::NodeStatus::SUCCESS;
  }

  static BT::PortsList providedPorts()
  {
    return { 
        BT::InputPort<Position2D>("pos_in", {0.0, 0.0}, "Initial position"),
        BT::InputPort<double>("x"),
        BT::InputPort<double>("y"), 
        BT::OutputPort<Position2D>("pos_out") 
    };
  }

private:
};

// Simple Action that updates an instance of Position2D in the blackboard
class FakeMoveBase : public BT::SyncActionNode
{
public:
  FakeMoveBase(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config)
  {}

  BT::NodeStatus tick() override
  {
    const auto pos = getInput<Position2D>("pos");
    if(!pos.has_value())
      return BT::NodeStatus::FAILURE;
    std::cout << "Moved to " << std::to_string(pos.value().x) << ", " << std::to_string(pos.value().y) << std::endl;
    return BT::NodeStatus::SUCCESS;
  }

  static BT::PortsList providedPorts()
  {
    return { 
        BT::InputPort<Position2D>("pos")
    };
  }

private:
};

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<BT::AccessJsonFieldNode>("AccessJsonField");

    factory.registerNodeType<BT::ForEachLoopNode<nlohmann::json>>("ForEachLoopJson");
  
    BT::RegisterJsonDefinition<Position2D>();
    factory.registerNodeType<FakeMoveBase>("FakeMoveBase");
    factory.registerNodeType<UpdatePosition>("UpdatePosition");

    factory.registerNodeType<BT::SetBlackboardEutNode>("SetBlackboardEut");

    factory.registerNodeType<BT::RandomValueNode<int>>("RandomInt");
    factory.registerNodeType<BT::RandomValueNode<uint32_t>>("RandomUInt");
    factory.registerNodeType<BT::RandomValueNode<double>>("RandomDouble");
    factory.registerNodeType<BT::RandomizeSequenceNode>("RandomSequenceValue");

    factory.registerNodeType<BT::ConcatenateStringsNode<2>>("ConcatenateStrings");
    factory.registerNodeType<BT::ConcatenateStringsNode<3>>("Concatenate3Strings");
    factory.registerNodeType<BT::ConcatenateStringsNode<4>>("Concatenate4Strings");
    factory.registerNodeType<BT::ConcatenateStringsNode<5>>("Concatenate5Strings");

    factory.registerNodeType<BT::ConvertJsonToNode<std::string>>("ConvertJsonToString");
    factory.registerNodeType<BT::ConvertJsonToNode<double>>("ConvertJsonToDouble");
    factory.registerNodeType<BT::ConvertJsonToNode<int16_t>>("ConvertJsonToShort");
    factory.registerNodeType<BT::ConvertJsonToNode<int32_t>>("ConvertJsonToInt32");
    factory.registerNodeType<BT::ConvertJsonToNode<int64_t>>("ConvertJsonToLong");
    factory.registerNodeType<BT::ConvertJsonToNode<uint16_t>>("ConvertJsonToUShort");
    factory.registerNodeType<BT::ConvertJsonToNode<uint32_t>>("ConvertJsonToUInt32");
    factory.registerNodeType<BT::ConvertJsonToNode<uint64_t>>("ConvertJsonToULong");

    factory.registerNodeType<BT::GetSizeNode<nlohmann::json, size_t>>("GetJsonSize");
    factory.registerNodeType<BT::GetSizeNode<nlohmann::json, uint32_t>>("GetJsonSizeUInt");
    factory.registerNodeType<BT::AddKeyValueToJson>("AddKeyValueToJson");
    factory.registerNodeType<BT::InitializeNode<nlohmann::json>>("InitializeJson");
    factory.registerNodeType<BT::AddArrayToJson>("AddArrayToJson");
    factory.registerNodeType<BT::SplitStringToJsonArray>("SplitStringToJsonArray");
    factory.registerNodeType<BT::CheckEmptyJson>("CheckEmptyJson");
    factory.registerNodeType<BT::CheckBoolNode>("CheckBool");

    factory.registerNodeType<BT::ComparisonNode<bool>>("CompareBool");
    factory.registerNodeType<BT::ComparisonNode<int16_t>>("CompareShort");
    factory.registerNodeType<BT::ComparisonNode<int32_t>>("CompareInt");
    factory.registerNodeType<BT::ComparisonNode<int64_t>>("CompareLong");
    factory.registerNodeType<BT::ComparisonNode<uint16_t>>("CompareUShort");
    factory.registerNodeType<BT::ComparisonNode<uint32_t>>("CompareUInt");
    factory.registerNodeType<BT::ComparisonNode<uint64_t>>("CompareULong");
    factory.registerNodeType<BT::ComparisonNode<double>>("CompareNumbers");
    factory.registerNodeType<BT::ComparisonNode<std::string>>("CompareStrings");
}
