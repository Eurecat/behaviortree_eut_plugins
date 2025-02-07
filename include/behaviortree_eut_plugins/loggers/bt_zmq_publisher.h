#ifndef BT_ZMQ_PUBLISHER_H
#define BT_ZMQ_PUBLISHER_H

#include <array>
#include <future>
#include "behaviortree_cpp/loggers/abstract_logger.h"

#include "behaviortree_eut_plugins/eut_debug.h"
#include "behaviortree_eut_plugins/flatbuffers/bt_flatbuffer_helper_eut.h"

namespace BT
{

class PublisherZMQ : public StatusChangeLogger
{
  static std::atomic<bool> ref_count;

  public:
    PublisherZMQ(BT::DebuggableTree& debugTree, unsigned max_msg_per_second = 25,
                unsigned publisher_port = 1666, unsigned server_port = 1667, const bool clean_output = true);

    virtual ~PublisherZMQ();

  private:
      // extract value from payload tokens given the name of the key
    static Expected<std::string> extractValue(const std::vector<StringView>& req_parts, const std::string& key = "");
    // extract debug command from request string
    static DebugCmd extractDebugCmd(const char* req);
    // extract debug payload from request string
    static std::shared_ptr<DebugPayload> extractDebugPayload(const char* req);

    virtual void callback(Duration timestamp, const TreeNode& node, NodeStatus prev_status,
                          NodeStatus status) override;

    virtual void debugCallback(Duration timestamp, const TreeNode& node, NodeAdvancedStatus prev_status,
                          NodeAdvancedStatus status);
    
    void updateTransitionBuffer(SerializedTransition&& transition, SerializedTransitionMaps&& serialized_transition_maps, const bool send_updates_if_possible = true);

    virtual void flush() override;

    std::shared_ptr<BT::Tree> tree_;
    std::vector<uint8_t> tree_buffer_;
    std::vector<uint8_t> status_buffer_;
    std::vector<SerializedTransition> transition_buffer_;
    std::vector<SerializedTransitionMaps> transition_maps_buffer_;
    std::chrono::microseconds min_time_between_msgs_;

    std::atomic_bool active_server_;
    std::thread thread_;

    void createStatusBuffer();

    TimePoint deadline_;
    std::mutex mutex_;
    std::atomic_bool send_pending_;
    std::condition_variable send_condition_variable_;
    std::future<void> send_future_;


    BT::TimePoint dfirst_timestamp_ = {};
    std::vector<DebugStatusChangeSubscriber> dsubscribers_;

    struct Pimpl;
    Pimpl* zmq_;
    const bool clean_output_;
  };
}   // namespace BT

#endif   // BT_ZMQ_PUBLISHER_H
