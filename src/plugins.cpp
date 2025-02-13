#include "behaviortree_cpp/bt_factory.h"

#include "behaviortree_eut_plugins/actions/RandomValueNode.h"
#include "behaviortree_eut_plugins/actions/WaitNode.h"



BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<BT::RandomValueNode<int>>("RandomInt");
  factory.registerNodeType<BT::RandomValueNode<uint32_t>>("RandomUInt");
  factory.registerNodeType<BT::RandomValueNode<double>>("RandomDouble");
  factory.registerNodeType<BT::WaitNode<std::chrono::seconds>>("WaitSeconds");
  factory.registerNodeType<BT::WaitNode<std::chrono::milliseconds>>("WaitMilliseconds");
}
