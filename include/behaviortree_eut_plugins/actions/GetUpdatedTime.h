#ifndef GET_UPDATED_TIME_NODE_HPP
#define GET_UPDATED_TIME_NODE_HPP

#include <behaviortree_cpp/action_node.h>
#include <chrono>

namespace BT
{
class GetUpdatedTime final : public BT::SyncActionNode
{
    public:
        using BT::SyncActionNode::SyncActionNode;
        GetUpdatedTime(const std::string& name, const BT::NodeConfiguration& config)
            : BT::SyncActionNode(name, config) 
            {}

        ~GetUpdatedTime() = default;

        static BT::PortsList providedPorts()
        {
            return { BT::InputPort<bool>("reset", false, "Reset clock"),
                     BT::OutputPort<uint32_t>("current_time", "Current time") };
        }

        virtual BT::NodeStatus tick() override
        {
            const auto& input_reset = getInput<bool>("reset");
            if (input_reset && input_reset.value()) 
            {
                start_time_ = std::chrono::steady_clock::now();
                // std::cout << "[GetUpdatedTime] start_time_ reset" << std::endl;

            }
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time_).count();
            // std::cout << "[GetUpdatedTime] now: " << std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count()
            //   << " ms, start_time: " << std::chrono::duration_cast<std::chrono::milliseconds>(start_time_.time_since_epoch()).count()
            //   << " ms, elapsed: " << elapsed << " ms" << std::endl;
            
              setOutput("current_time", static_cast<uint32_t>(elapsed));
            return BT::NodeStatus::SUCCESS;
        }
    private:
        static std::chrono::steady_clock::time_point start_time_;
};
}

#endif
