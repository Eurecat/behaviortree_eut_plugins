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
        throw RuntimeError("Can't find the port referred by [value]");
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

    bool dst_entry_anytype_allowed = dst_entry && dst_entry->info.type() == typeid(BT::AnyTypeAllowed); 
    bool dst_entry_strongly_typed = dst_entry && dst_entry->info.isStronglyTyped();

    if(dst_entry_anytype_allowed && out_value.isString())
    {
      std::scoped_lock scoped_lock(dst_entry->entry_mutex);
      dst_entry->sequence_id++;
      dst_entry->stamp = std::chrono::steady_clock::now().time_since_epoch();
      dst_entry->value = std::move(out_value);
    }
    else
    {
      // avoid type issues when port is remapped: current implementation of the set might be a little bit problematic for initialized on the fly values
      // this still does not attack math issues
      if(dst_entry_strongly_typed && dst_entry->info.type() != typeid(std::string) && out_value.isString())
      {
        try
        {
          out_value = dst_entry->info.parseString(out_value.cast<std::string>());
        }
        catch(const std::exception& e)
        {
          throw LogicError("Can't convert string [", out_value.cast<std::string>(),
                          "] to type [", BT::demangle(dst_entry->info.type()),
                          "]: ", e.what());
        }
      }
      config().blackboard->set(output_key, out_value);
    }

    return NodeStatus::SUCCESS;
  }
};
}  // namespace BT

#endif
