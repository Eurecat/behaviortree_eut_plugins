#ifndef WAIT_NODE_HPP
#define WAIT_NODE_HPP

#ifndef BT_NO_COROUTINES

#include <chrono>
#include "behaviortree_cpp/action_node.h"

namespace BT
{
template <class T>
class WaitNode final : public BT::CoroActionNode
{
    template <typename R>
    struct isDuration : std::false_type {};

    template <typename Rep, typename Period>
    struct isDuration<std::chrono::duration<Rep, Period>> : std::true_type {};

    static_assert(isDuration<T>::value, "WaitNode template argument must be a std::chrono::duration");

    public:
        using BT::CoroActionNode::CoroActionNode;
        ~WaitNode() = default;

        static BT::PortsList providedPorts()
        {
            return { BT::InputPort<unsigned>("duration", 0, "Time to wait") };
        }

        virtual BT::NodeStatus tick() override
        {
            if(!waiting_)
            {
                start_waiting_time_ = std::chrono::system_clock::now();

                const auto& wait_duration_raw = getInput<unsigned>("duration");
                if(!wait_duration_raw) { throw BT::RuntimeError { name() + ": " + wait_duration_raw.error() }; }

                wait_duration_ = T { wait_duration_raw.value() };
                waiting_ = true;
            }

            while(waiting_ && std::chrono::duration_cast<T>(std::chrono::system_clock::now() - start_waiting_time_) < wait_duration_)
            {
                setStatusRunningAndYield();
            }

            waiting_ = false;
            return BT::NodeStatus::SUCCESS;
        }

        virtual void halt() override
        {
            waiting_ = false;
            CoroActionNode::halt();
        }

    private:
        bool waiting_ { false };
        std::chrono::system_clock::time_point start_waiting_time_;
        T wait_duration_;
};
}
#endif

#endif
