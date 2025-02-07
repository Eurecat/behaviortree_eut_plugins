#include "behaviortree_cpp/loggers/bt_file_logger_v2.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "behaviortree_cpp/xml_parsing.h"
#include "behaviortree_cpp/json_export.h"
#include "behaviortree_cpp/loggers/bt_minitrace_logger.h"
#include "behaviortree_cpp/json_export.h"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "behaviortree_eut_plugins/eut_utils.h"
#include "behaviortree_eut_plugins/eut_debug.h"
#include "behaviortree_eut_plugins/loggers/bt_zmq_publisher.h"

#include "behaviortree_eut_plugins/loggers/bt_file_logger.h"


class CrossDoor
{
public:
  void registerNodes(BT::BehaviorTreeFactory& factory);

  void reset();

  // SUCCESS if _door_open == true
  BT::NodeStatus isDoorClosed();

  // SUCCESS if _door_open == true
  BT::NodeStatus passThroughDoor();

  // After 3 attempts, will open a locked door
  BT::NodeStatus pickLock();

  // FAILURE if door locked
  BT::NodeStatus openDoor();

  // WILL always open a door
  BT::NodeStatus smashDoor();

private:
  bool _door_open = false;
  bool _door_locked = true;
  int _pick_attempts = 0;
};

inline void SleepMS(int ms)
{
  std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

using BT::NodeStatus;

NodeStatus CrossDoor::isDoorClosed()
{
  SleepMS(200);
  return !_door_open ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
}

NodeStatus CrossDoor::passThroughDoor()
{
  SleepMS(500);
  return _door_open ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
}

NodeStatus CrossDoor::openDoor()
{
  SleepMS(500);
  if(_door_locked)
  {
    return NodeStatus::FAILURE;
  }
  else
  {
    _door_open = true;
    return NodeStatus::SUCCESS;
  }
}

NodeStatus CrossDoor::pickLock()
{
  SleepMS(500);
  // succeed at 3rd attempt
  if(_pick_attempts++ > 3)
  {
    _door_locked = false;
    _door_open = true;
  }
  return _door_open ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
}

NodeStatus CrossDoor::smashDoor()
{
  _door_locked = false;
  _door_open = true;
  // smash always works
  return NodeStatus::SUCCESS;
}

void CrossDoor::registerNodes(BT::BehaviorTreeFactory& factory)
{
  factory.registerSimpleCondition("IsDoorClosed",
                                  std::bind(&CrossDoor::isDoorClosed, this));

  factory.registerSimpleAction("PassThroughDoor",
                               std::bind(&CrossDoor::passThroughDoor, this));

  factory.registerSimpleAction("OpenDoor", std::bind(&CrossDoor::openDoor, this));

  factory.registerSimpleAction("PickLock", std::bind(&CrossDoor::pickLock, this));

  factory.registerSimpleCondition("SmashDoor", std::bind(&CrossDoor::smashDoor, this));
}

void CrossDoor::reset()
{
  _door_open = false;
  _door_locked = true;
  _pick_attempts = 0;
}


/** We are using the same example in Tutorial 5,
 *  But this time we also show how to connect
 */

// A custom struct  that I want to visualize in Groot
struct Position2D
{
  double x;
  double y;
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

template <>
[[nodiscard]] std::string BT::toStr(const Position2D& value)
{
  return "x: " + std::to_string(value.x) + "; y: " + std::to_string(value.y); 
}

// Simple Action that updates an instance of Position2D in the blackboard
class UpdatePosition : public BT::SyncActionNode
{
public:
  UpdatePosition(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config)
  {}

  BT::NodeStatus tick() override
  {
    const auto testGetAsJson = BT::getPortValueAsJson(*this, "pos", BT::PortDirection::OUTPUT);
    if(testGetAsJson)
      std::cout << "getOutputAsJson: " << testGetAsJson.value().dump() << "\n" << std::flush;
    _pos.x += 0.2;
    _pos.y += 0.1;
    setOutput("pos", _pos);
    return BT::NodeStatus::SUCCESS;
  }

  static BT::PortsList providedPorts()
  {
    return { BT::OutputPort<Position2D>("pos") };
  }

private:
  Position2D _pos = { 0, 0 };
};

// Simple Action that updates an instance of Position2D in the blackboard
class LogString : public BT::SyncActionNode
{
public:
  LogString(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config)
  {}

  BT::NodeStatus tick() override
  { 
    std::cout << "\n\nLogString\n" << std::flush;
    const auto testGetAsJson = BT::getPortValueAsJson(*this, "something", BT::PortDirection::INPUT);
    if(testGetAsJson)
      std::cout << "getInputAsJson (something): " << testGetAsJson.value().dump() << " (json_type = " << testGetAsJson.value().type_name() << ") \n" << std::flush;
    bool fail = false;
    const auto it = config().input_ports.find("something");
    if(it != config().input_ports.end())
    {
      if(BT::TreeNode::isBlackboardPointer(it->second))
      {
        std::cout << "something port value is key " << std::string(BT::TreeNode::stripBlackboardPointer(it->second)) << "\n" << std::flush;
        std::shared_ptr<BT::Blackboard::Entry> entry = config().blackboard->getEntry(std::string(BT::TreeNode::stripBlackboardPointer(it->second)));
        fail = (!entry);
        if(!fail)
        {
          nlohmann::json json;
          BT::JsonExporter::get().toJson(entry->value, json);
          std::cout << "VALUE OF something (BB key=" << it->second << "): " << json.dump() << "\n" << std::flush;
        }
      }
      else
        std::cout << "VALUE OF something: " << it->second << "\n" << std::flush;

    }
    else fail = true;

    return fail? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<void>("something") };
  }

private:
  Position2D _pos = { 0, 0 };
};

// Simple Action that updates an instance of Position2D in the blackboard
class LogBool : public BT::SyncActionNode
{
public:
  LogBool(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config)
  {}

  BT::NodeStatus tick() override
  { 
    std::cout << "\n\nLogBool\n" << std::flush;
    const auto testGetAsJson = BT::getPortValueAsJson(*this, "val");
    if(testGetAsJson)
      std::cout << "LogBool getInputAsJson (val): " << testGetAsJson.value().dump() << " (json_type = " << testGetAsJson.value().type_name() << ") \n" << std::flush;
    bool fail = false;

    return fail? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<bool>("val") };
  }

private:
};
// clang-format off

// static const char* xml_text = R"(
// <root BTCPP_format="4">

//   <BehaviorTree ID="MainTree">
//     <Sequence>
//       <Script code="door_open:=false" />
//     </Sequence>
//   </BehaviorTree>

// </root>
//  )";

// clang-format on

int main()
{
  BT::BehaviorTreeFactory factory;

  // Nodes registration, as usual
  CrossDoor cross_door;
  cross_door.registerNodes(factory);
  factory.registerNodeType<UpdatePosition>("UpdatePosition");
  factory.registerNodeType<LogString>("LogString");
  factory.registerNodeType<LogBool>("LogBool");

  // Groot2 editor requires a model of your registered Nodes.
  // You don't need to write that by hand, it can be automatically
  // generated using the following command.
  const std::string xml_models = BT::writeTreeNodesModelXML(factory);

  // factory.registerBehaviorTreeFromText(xml_text);

  // Add this to allow Groot to visualize your custom type
  BT::RegisterJsonDefinition<Position2D>();
  
  // Get the share directory of the package
  std::string package_share_dir = ament_index_cpp::get_package_share_directory("behaviortree_eut_plugins");

  // Construct the path to the XML file
  std::string tree_file_path = package_share_dir + "/examples/trees/t1.xml";

  std::shared_ptr<BT::Tree> tree_ptr = std::make_shared<BT::Tree>(factory.createTreeFromFile(tree_file_path));
  BT::DebuggableTree debugTree{tree_ptr};

  BT::Tree& tree = *(tree_ptr); // you don't really need it, but let's assume you wanna have a direct ref. to the object

  std::cout << "----------- XML file  ----------\n"
            << BT::WriteTreeToXML(tree, false, false)
            << "--------------------------------\n";
  
  // Connect the PublisherZMQ. This will allow GrootEUT to
  // get the tree and poll status updates.
  const unsigned port = 1667;
  BT::PublisherZMQ publisher(debugTree, port);

  BT::FileLogger logger(tree, "devistree.fbl", 10, true);

  // Add two more loggers, to save the transitions into a file.
  // Both formats are compatible with Groot2

  // Logging with lightweight serialization
  // BT::FileLogger2 logger2(tree, "t12_logger2.btlog");
  // BT::MinitraceLogger minilog(tree, "minitrace.json");

  // char c;
  // std::cout << "Any key and enter to start ticking: ";
  // std::cin >> c;
  while(1)
  {
    std::cout << "Start" << std::endl;
    cross_door.reset();
    tree.tickWhileRunning();
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  }

  return 0;
}
