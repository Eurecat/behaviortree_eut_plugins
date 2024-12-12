#include "behavior_tree_eut_plugins/loggers/bt_file_logger.h"

#include "behavior_tree_eut_plugins/eut_debug.h"

namespace BT
{
FileLogger::FileLogger(const BT::Tree& tree, const char* filename, uint16_t buffer_size, const bool store_values, const bool clean_output) :
  StatusChangeLogger(tree.rootNode()), buffer_max_size_(buffer_size), store_values_(store_values), clean_output_(clean_output)
{
  if (buffer_max_size_ != 0)
  {
    buffer_.reserve(buffer_max_size_);
    if(store_values) transition_maps_buffer_.reserve(buffer_max_size_);
  }

  enableTransitionToIdle(true);

  flatbuffers::FlatBufferBuilder builder(1024);
  CreateFlatbuffersBehaviorTree(builder, tree);

  //-------------------------------------

  file_os_.open(filename, std::ofstream::binary | std::ofstream::out);

  // serialize the length of the buffer in the first 4 bytes
  char size_buff[4];
  flatbuffers::WriteScalar(size_buff, static_cast<int32_t>(builder.GetSize()));

  file_os_.write(size_buff, 4);
  file_os_.write(reinterpret_cast<const char*>(builder.GetBufferPointer()),
                 builder.GetSize());
  
  // serialize fake transition at start with invalid values to flag Groot about it
  if(store_values_)
  {
    SerializedTransition serialized_transition = CreateInvalidSerializedTransition();
    file_os_.write(reinterpret_cast<const char*>(serialized_transition.data()), serialized_transition.size());
  }
}

FileLogger::~FileLogger()
{
  this->flush();
  file_os_.close();
  buffer_.clear();
  if(store_values_) transition_maps_buffer_.clear();
}

void FileLogger::callback(Duration timestamp, const TreeNode& node,
                          NodeStatus prev_status, NodeStatus status)
{
  SerializedTransition serialized_transition =
      SerializeTransition(node.UID(), timestamp, prev_status, status);

  SerializedTransitionMaps serialized_transition_maps = 
    SerializeTransitionMaps(node.UID(), timestamp, 
      (prev_status != BT::NodeStatus::IDLE && status == BT::NodeStatus::IDLE)? (BT::PortsValueMap{}) : (getPortValuesMap(node, PortDirection::INPUT, clean_output_)), 
      (prev_status != BT::NodeStatus::IDLE && status == BT::NodeStatus::IDLE)? (BT::PortsValueMap{}) : (getPortValuesMap(node, PortDirection::OUTPUT, clean_output_)), 
      /*node.getBlackboardValuesMap()*/{}); // TODO: review bb serialization protocol: for now avoid to send within the msg the entire BB dump: overkill and barely used on the other side
  
  // if(node.type() == BT::NodeType::SUBTREE)
  // {
  //   try 
  //   {
  //     const SubTreeNode& subtree_node = dynamic_cast<const SubTreeNode&>(node);
  //     // add manually for subtree port remappings
  //     serialized_transition_maps = SerializeTransitionMaps(node.UID(), timestamp, 
  //       (prev_status != BT::NodeStatus::IDLE && status == BT::NodeStatus::IDLE)? (BT::PortsValueMap{}) : (getPortValuesMap(subtree_node, PortDirection::INPUT, clean_output_)));
  //   }
  //   catch(const std::bad_cast& e) 
  //   {
  //       // Cast failed
  //   }
  // }

  if (buffer_max_size_ == 0)
  {
    file_os_.write(reinterpret_cast<const char*>(serialized_transition.data()), serialized_transition.size());
    if(store_values_) 
    {
        writeSerializedTransitionMaps(serialized_transition_maps);
    }
  }
  else
  {
    buffer_.push_back(serialized_transition);
    transition_maps_buffer_.push_back(serialized_transition_maps);
    if (buffer_.size() >= buffer_max_size_)
    {
      this->flush();
    }
  }
}

void FileLogger::writeSerializedTransitionMaps(const SerializedTransitionMaps& serialized_trans_tmaps)
{
  if(store_values_ && file_os_.is_open())
  {
    const std::string flatten_serialized_transition_maps = flattenSerializeTransitionMaps(serialized_trans_tmaps);
    size_t serialized_maps_size = flatten_serialized_transition_maps.length() * sizeof(char);

    char size_ser_maps[4];
    flatbuffers::WriteScalar<int32_t>(size_ser_maps, static_cast<int32_t>(serialized_maps_size));
    file_os_.write(size_ser_maps, sizeof(int32_t));
    file_os_.write(reinterpret_cast<const char*>(flatten_serialized_transition_maps.data()), serialized_maps_size);
  }
}

void FileLogger::flush()
{
  for (std::size_t i = 0; i<buffer_.size(); i++)
  {
    const auto& serialized_transition = buffer_[i];
    file_os_.write(reinterpret_cast<const char*>(serialized_transition.data()), serialized_transition.size());
    if(store_values_ && i<transition_maps_buffer_.size())
    {
        writeSerializedTransitionMaps(transition_maps_buffer_[i]);
    }
  }
  file_os_.flush();
  buffer_.clear();
  if(store_values_) transition_maps_buffer_.clear();
}
}   // namespace BT
