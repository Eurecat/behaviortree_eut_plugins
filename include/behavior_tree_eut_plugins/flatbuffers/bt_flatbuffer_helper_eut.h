#ifndef BT_FLATBUFFER_HELPER_H
#define BT_FLATBUFFER_HELPER_H

#include "behaviortree_cpp/bt_factory.h"
#include "BT_logger_generated.h"
#include <boost/algorithm/string/replace.hpp>

namespace BT
{
typedef std::unordered_map<std::string, std::string> PortsValueMap;

typedef std::array<uint8_t, 12> SerializedTransition;
typedef std::array<std::string, 4> SerializedTransitionMaps;

inline Serialization::NodeType convertToFlatbuffers(BT::NodeType type)
{
  switch (type)
  {
    case BT::NodeType::ACTION:
      return Serialization::NodeType::ACTION;
    case BT::NodeType::DECORATOR:
      return Serialization::NodeType::DECORATOR;
    case BT::NodeType::CONTROL:
      return Serialization::NodeType::CONTROL;
    case BT::NodeType::CONDITION:
      return Serialization::NodeType::CONDITION;
    case BT::NodeType::SUBTREE:
      return Serialization::NodeType::SUBTREE;
    case BT::NodeType::UNDEFINED:
      return Serialization::NodeType::UNDEFINED;
  }
  return Serialization::NodeType::UNDEFINED;
}

inline Serialization::NodeStatus convertToFlatbuffers(BT::NodeStatus type)
{
  switch (type)
  {
    case BT::NodeStatus::SKIPPED:
      return Serialization::NodeStatus::SKIPPED;
    // case BT::NodeStatus::PAUSED:
    //   return Serialization::NodeStatus::PAUSED;
    case BT::NodeStatus::IDLE:
      return Serialization::NodeStatus::IDLE;
    case BT::NodeStatus::SUCCESS:
      return Serialization::NodeStatus::SUCCESS;
    case BT::NodeStatus::RUNNING:
      return Serialization::NodeStatus::RUNNING;
    case BT::NodeStatus::FAILURE:
      return Serialization::NodeStatus::FAILURE;
  }
  return Serialization::NodeStatus::IDLE;
}

inline Serialization::NodeAdvancedStatus convertToFlatbuffers(BT::NodeAdvancedStatus type)
{
  switch (type)
  {
    case BT::NodeAdvancedStatus::SKIPPED:
      return Serialization::NodeAdvancedStatus::SKIPPED;
    case BT::NodeAdvancedStatus::PAUSED:
      return Serialization::NodeAdvancedStatus::PAUSED;
    case BT::NodeAdvancedStatus::IDLE:
      return Serialization::NodeAdvancedStatus::IDLE;
    case BT::NodeAdvancedStatus::SUCCESS:
      return Serialization::NodeAdvancedStatus::SUCCESS;
    case BT::NodeAdvancedStatus::RUNNING:
      return Serialization::NodeAdvancedStatus::RUNNING;
    case BT::NodeAdvancedStatus::FAILURE:
      return Serialization::NodeAdvancedStatus::FAILURE;
  }
  return Serialization::NodeAdvancedStatus::IDLE;
}

inline Serialization::PortDirection convertToFlatbuffers(BT::PortDirection direction)
{
  switch (direction)
  {
    case BT::PortDirection::INPUT:
      return Serialization::PortDirection::INPUT;
    case BT::PortDirection::OUTPUT:
      return Serialization::PortDirection::OUTPUT;
    case BT::PortDirection::INOUT:
      return Serialization::PortDirection::INOUT;
  }
  return Serialization::PortDirection::INOUT;
}

inline void CreateFlatbuffersBehaviorTree(flatbuffers::FlatBufferBuilder& builder,
                                          const BT::Tree& tree, const bool debuggable = false, std::set<uint16_t> bp_nodes = {}, std::pair<bool, uint16_t> paused_uid = {false, 0})
{
  std::vector<flatbuffers::Offset<Serialization::TreeNode>> fb_nodes;
  applyRecursiveVisitor(tree.rootNode(), [&](BT::TreeNode* node) {
    
    const BT::TreeNode& node_const_ref = *node;

    std::vector<uint16_t> children_uid;
    if (auto control = dynamic_cast<BT::ControlNode*>(node))
    {
      children_uid.reserve(control->children().size());
      for (const auto& child : control->children())
      {
        children_uid.push_back(child->UID());
      }
    }
    else if (auto decorator = dynamic_cast<BT::DecoratorNode*>(node))
    {
      const auto& child = decorator->child();
      children_uid.push_back(child->UID());
    }

    std::vector<flatbuffers::Offset<Serialization::PortConfig>> ports;
    
    // if(auto subtree = dynamic_cast<BT::SubTreeNode*>(node))
    // {
    //   const BT::SubTreeNode& subtreenode_const_ref = *subtree;
    //   for (const auto& it : subtreenode_const_ref.config().input_ports)
    //   {  
    //     // You can treat them as if they were ports on serialization
    //     ports.push_back(Serialization::CreatePortConfigDirect(builder, it.first.c_str(),
    //                                                           it.second.c_str()));
    //   }
    // }
    // else
    {
      for (const auto& it : node_const_ref.config().pre_conditions)
      {
        ports.push_back(Serialization::CreatePortConfigDirect(builder, toStr<BT::PreCond>(it.first).c_str(),
                                                              it.second.c_str()));
      }
      for (const auto& it : node_const_ref.config().input_ports)
      {
        ports.push_back(Serialization::CreatePortConfigDirect(builder, it.first.c_str(),
                                                              it.second.c_str()));
      }
      for (const auto& it : node_const_ref.config().output_ports)
      {
        ports.push_back(Serialization::CreatePortConfigDirect(builder, it.first.c_str(),
                                                              it.second.c_str()));
      }

      for (const auto& it : node_const_ref.config().post_conditions)
      {
        ports.push_back(Serialization::CreatePortConfigDirect(builder, toStr<BT::PostCond>(it.first).c_str(),
                                                              it.second.c_str()));
      }
    }

    BT::NodeAdvancedStatus adv_status = toNodeAdvancedStatus(node->status()); 
    if(paused_uid.first && paused_uid.second == node->UID()) adv_status = NodeAdvancedStatus::PAUSED;

    auto tn = Serialization::CreateTreeNode(
        builder, node->UID(), builder.CreateVector(children_uid),
        convertToFlatbuffers(adv_status), builder.CreateString(node->name().c_str()),
        builder.CreateString(node->registrationName().c_str()),
        builder.CreateVector(ports), false, bp_nodes.count(node->UID())); //TODO Devis
        // node->disabled(), node_const_ref.config().breakpoint);

    fb_nodes.push_back(tn);
  });

  std::vector<flatbuffers::Offset<Serialization::NodeModel>> node_models;

  for (const auto& node_it : tree.manifests)
  {
    const auto& manifest = node_it.second;
    std::vector<flatbuffers::Offset<Serialization::PortModel>> port_models;

    for (const auto& port_it : manifest.ports)
    {
      const auto& port_name = port_it.first;
      const auto& port = port_it.second;
      auto port_model = Serialization::CreatePortModel(
          builder, builder.CreateString(port_name.c_str()),
          convertToFlatbuffers(port.direction()),
          builder.CreateString(demangle(port.type()).c_str()),
          builder.CreateString(port.description().c_str()));
      port_models.push_back(port_model);
    }

    auto node_model = Serialization::CreateNodeModel(
        builder, builder.CreateString(manifest.registration_ID.c_str()),
        convertToFlatbuffers(manifest.type), builder.CreateVector(port_models));

    node_models.push_back(node_model);
  }

  auto behavior_tree = Serialization::CreateBehaviorTree(
      builder, tree.rootNode()->UID(), debuggable, 
      builder.CreateVector(fb_nodes),
      builder.CreateVector(node_models));

  builder.Finish(behavior_tree);
}

/** Serialize manually the informations about state transition
 * No flatbuffer serialization here
 */
inline SerializedTransition SerializeTransition(uint16_t UID, Duration timestamp,
                                                NodeStatus prev_status, NodeStatus status)
{
  using namespace std::chrono;
  SerializedTransition buffer;
  int64_t usec = duration_cast<microseconds>(timestamp).count();
  int64_t t_sec = usec / 1000000;
  int64_t t_usec = usec % 1000000;

  flatbuffers::WriteScalar(&buffer[0], t_sec);
  flatbuffers::WriteScalar(&buffer[4], t_usec);
  flatbuffers::WriteScalar(&buffer[8], UID);

  flatbuffers::WriteScalar(&buffer[10],
                           static_cast<int8_t>(convertToFlatbuffers(prev_status)));
  flatbuffers::WriteScalar(&buffer[11],
                           static_cast<int8_t>(convertToFlatbuffers(status)));

  return buffer;
}

/** Serialize manually the informations about state transition
 * No flatbuffer serialization here
 */
inline SerializedTransition SerializeTransition(uint16_t UID, Duration timestamp,
                                                NodeAdvancedStatus prev_status, NodeAdvancedStatus status)
{
  using namespace std::chrono;
  SerializedTransition buffer;
  int64_t usec = duration_cast<microseconds>(timestamp).count();
  int64_t t_sec = usec / 1000000;
  int64_t t_usec = usec % 1000000;

  flatbuffers::WriteScalar(&buffer[0], t_sec);
  flatbuffers::WriteScalar(&buffer[4], t_usec);
  flatbuffers::WriteScalar(&buffer[8], UID);

  flatbuffers::WriteScalar(&buffer[10],
                           static_cast<int8_t>(convertToFlatbuffers(prev_status)));
  flatbuffers::WriteScalar(&buffer[11],
                           static_cast<int8_t>(convertToFlatbuffers(status)));

  return buffer;
}

/** 
 * Serialize manually the informations about an invalid state transition -> used as a tag
 * No flatbuffer serialization here
 */
inline SerializedTransition CreateInvalidSerializedTransition()
{
  using namespace std::chrono;
  SerializedTransition buffer;

  flatbuffers::WriteScalar(&buffer[0], 0);
  flatbuffers::WriteScalar(&buffer[4], 0);
  flatbuffers::WriteScalar(&buffer[8], 0);

  flatbuffers::WriteScalar(&buffer[10],
                           static_cast<int8_t>(-1));
  flatbuffers::WriteScalar(&buffer[11],
                           static_cast<int8_t>(-1));

  return buffer;
}

inline std::string flattenValueMap(const PortsValueMap& valuesmap){
  std::string res = "";
  for(auto it = valuesmap.begin(); it!=valuesmap.end(); it++)
  {
    if(it!=valuesmap.begin())
      res += ",";
    
    // Insert escape characters to not confuse any brackets in the key/value pairs with the brackets delimiting the flatten map
    auto key = boost::replace_all_copy(it->first, "{", "\\{");
    key = boost::replace_all_copy(key, "}", "\\}");
    auto value = boost::replace_all_copy(it->second,  "{", "\\{");
    value = boost::replace_all_copy(value,  "}", "\\}");
    res += "{"+key+":"+value+"}";

  }
  return res;
}

/** Serialize manually the informations about state transition
 * No flatbuffer serialization here
 */
inline SerializedTransitionMaps SerializeTransitionMaps(uint16_t UID, Duration timestamp, const PortsValueMap& inports_valmap, const PortsValueMap& outports_valmap, const PortsValueMap& bb_valmap)
{
  SerializedTransitionMaps buffer;
  if(inports_valmap.size() > 0 || outports_valmap.size() > 0 || bb_valmap.size() > 0)
  {
    using namespace std::chrono;
    int64_t usec = duration_cast<microseconds>(timestamp).count();
    int64_t t_sec = usec / 1000000;
    int64_t t_usec = usec % 1000000;
    buffer[0] = "UID:" + std::to_string(UID) + ",TS:" + std::to_string(t_sec) + "." + std::to_string(t_usec);
  }
  else
    buffer[0] = "";
  
  buffer[1] = inports_valmap.size()  > 0? "IN:{" + flattenValueMap(inports_valmap) + "}" : "";
  buffer[2] = outports_valmap.size() > 0? "OUT:{" + flattenValueMap(outports_valmap) + "}" : "";
  buffer[3] = bb_valmap.size() > 0?       "BB:{" + flattenValueMap(bb_valmap) + "}" : "";
  return buffer;
}

inline SerializedTransitionMaps SerializeTransitionMaps(uint16_t UID, Duration timestamp, const PortsValueMap& remappedports_valmap)
{
  SerializedTransitionMaps buffer;
  if(remappedports_valmap.size() > 0)
  {
    using namespace std::chrono;
    int64_t usec = duration_cast<microseconds>(timestamp).count();
    int64_t t_sec = usec / 1000000;
    int64_t t_usec = usec % 1000000;
    buffer[0] = "UID:" + std::to_string(UID) + ",TS:" + std::to_string(t_sec) + "." + std::to_string(t_usec);
  }
  else
    buffer[0] = "";

  buffer[1] = "";
  
  // TODO: treat port remappings separately in the future
  // add manually for subtree port remappings and put them as OUTPUT port for now (notice that INOUT ports are treated like this now)
  buffer[2] = remappedports_valmap.size() > 0? "OUT:{" + flattenValueMap(remappedports_valmap) + "}" : "";
  
  buffer[3] = "";

  return buffer;
}

/** Serialize manually the informations about state transition
 * No flatbuffer serialization here
 */
inline std::string flattenSerializeTransitionMaps(const SerializedTransitionMaps& maps)
{
  std::string res = "{";
  
  res += maps[0];
  res += (maps[1].length() > 0 && res.length() > 0 && res[res.length()-1] != ','? "," : "") + maps[1];
  res += (maps[2].length() > 0 && res.length() > 0 && res[res.length()-1] != ','? "," : "") + maps[2];
  res += (maps[3].length() > 0 && res.length() > 0 && res[res.length()-1] != ','? "," : "") + maps[3];
  
  res += "}";
  
  if(res == "{}")
    return "";//pointless to embed this information, UID and ts will be used for matching later

  return res;
}

}   // namespace BT

#endif   // BT_FLATBUFFER_HELPER_H
