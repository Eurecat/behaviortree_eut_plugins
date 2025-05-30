#ifndef ACTION_SETBLACKBOARD_EUT_NODE_H
#define ACTION_SETBLACKBOARD_EUT_NODE_H

#include "behaviortree_cpp/action_node.h"

namespace BT
{
/**
 * @brief The SetBlackboard is action used to store a string
 * into an entry of the Blackboard specified in "output_key".
 *
 * Example usage:
 *
 *  <SetBlackboard value="42" output_key="the_answer" />
 *
 * Will store the string "42" in the entry with key "the_answer".
 *
 * Alternatively, you can use it to copy one port inside another port:
 *
 * <SetBlackboard value="{src_port}" output_key="dst_port" />
 *
 * This will copy the type and content of {src_port} into {dst_port}
 */
class SetBlackboardEutNode : public SyncActionNode
{
public:
  SetBlackboardEutNode(const std::string& name, const NodeConfig& config)
    : SyncActionNode(name, config)
  {
    setRegistrationID("SetBlackboard");
  }

  static PortsList providedPorts()
  {
    return { InputPort("value", "Value to be written int othe output_key"),
             BidirectionalPort("output_key", "Name of the blackboard entry where the "
                                             "value should be written") };
  }

private:
  inline bool validSignedIntegralType(const std::type_index& type)
  {
    return type == typeid(int64_t) || type == typeid(int32_t) || type == typeid(int16_t) || 
    type == typeid(int8_t);
  }
  inline bool validUnsignedIntegralType(const std::type_index& type)
  {
    return type == typeid(uint64_t) || type == typeid(uint32_t) || type == typeid(uint16_t) ||
    type == typeid(uint8_t);
  }
  inline bool validIntegralType(const std::type_index& type)
  {
    return validSignedIntegralType(type) || validUnsignedIntegralType(type);
  }
  inline bool validFloatingPointType(const std::type_index& type)
  {
    return type == typeid(float) || type == typeid(double);
  }
  inline bool validNumberType(const std::type_index& type)
  {
    return validIntegralType(type) || validFloatingPointType(type);
  }

  virtual BT::NodeStatus tick() override
  {
    std::string output_key;
    if(!getInput("output_key", output_key))
    {
      throw RuntimeError("SetBlackboardEut: missing port [output_key]");
    }

    auto value_str_it = config().input_ports.find("value");
    if(value_str_it == config().input_ports.end())
    {
      throw RuntimeError("SetBlackboardEut: missing port [value]");
    }

    const std::string value_str = value_str_it->second;

    StringView stripped_key;
    BT::Any out_value;

    std::shared_ptr<Blackboard::Entry> dst_entry =
        config().blackboard->getEntry(output_key);

    if(isBlackboardPointer(value_str, &stripped_key))
    {
      const auto input_key = std::string(stripped_key);
      std::shared_ptr<Blackboard::Entry> src_entry =
          config().blackboard->getEntry(input_key);

      if(!src_entry)
      {
        throw RuntimeError("SetBlackboardEut: Can't find the port referred by [value]");
      }
      if(!dst_entry)
      {
        config().blackboard->createEntry(output_key, src_entry->info);
        dst_entry = config().blackboard->getEntry(output_key);
      }

      out_value = src_entry->value;
    }
    else
    {
      out_value = BT::Any(value_str);
    }

    if(out_value.empty())
      return NodeStatus::FAILURE;

    const std::type_index& dst_entry_info_type = dst_entry->info.type();
    bool dst_entry_anytype_allowed = dst_entry && dst_entry_info_type == typeid(BT::AnyTypeAllowed); 
    bool dst_entry_strongly_typed = dst_entry && dst_entry->info.isStronglyTyped();

    if(dst_entry_anytype_allowed && out_value.isString())
    {
      std::scoped_lock scoped_lock(dst_entry->entry_mutex);
      dst_entry->value = std::move(out_value);
      dst_entry->sequence_id++;
      dst_entry->stamp = std::chrono::steady_clock::now().time_since_epoch();
    }
    else
    {
      // avoid type issues when port is remapped: current implementation of the set might be a little bit problematic for initialized on the fly values
      if(dst_entry_strongly_typed &&  dst_entry_info_type != out_value.type()) // types are mismatching
      {
        if(out_value.isString() && dst_entry_info_type != typeid(std::string))// If the destination entry is strongly typed, we need to ensure that string values are converted to the correct type.
        {
          try
          {
            out_value = dst_entry->info.parseString(out_value.cast<std::string>());
          }
          catch(const std::exception& e)
          {
            throw LogicError("SetBlackboardEut: Can't convert string [", out_value.cast<std::string>(),
                            "] to type [", BT::demangle(dst_entry_info_type),
                            "]: ", e.what());
          }
        }
        else if(out_value.isNumber() && validNumberType(dst_entry_info_type)) // If the destination entry is strongly typed, we need to ensure that numeric values are converted to the correct type.
        {
          try
          {
            std::scoped_lock scoped_lock(dst_entry->entry_mutex);
            bool updated = false;
            if(out_value.isIntegral())
            {
              if(validSignedIntegralType(out_value.castedType()) && isCastingSafe(dst_entry_info_type, out_value.cast<int64_t>()))
              {
                dst_entry->value.copyInto(out_value);
                updated = true;
              }
              else if(validUnsignedIntegralType(out_value.castedType()) && isCastingSafe(dst_entry_info_type, out_value.cast<uint64_t>()))
              {
                dst_entry->value.copyInto(out_value);
                updated = true;
              }
            }
            else if(validFloatingPointType(out_value.castedType()) && isCastingSafe(dst_entry_info_type, out_value.cast<double>()))
            {
              dst_entry->value.copyInto(out_value);
              updated = true;
            }

            if(updated)
            {
              dst_entry->sequence_id++;
              dst_entry->stamp = std::chrono::steady_clock::now().time_since_epoch();
              return NodeStatus::SUCCESS;
            }

          }
          catch(const std::runtime_error& re)
          {
            throw LogicError("SetBlackboardEut: Can't convert value [", value_str,
                               "] to type [", BT::demangle(dst_entry_info_type), "]");
          }
        }

        // for all the other cases, we fallback to the default behavior of the Blackboard::set method which shall already deal correctly with the type mismatch
      }
      config().blackboard->set(output_key, out_value);
    }

    return NodeStatus::SUCCESS;
  }
};
}  // namespace BT

#endif
