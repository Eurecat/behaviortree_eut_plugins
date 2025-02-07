#ifndef BT_FILE_LOGGER_H
#define BT_FILE_LOGGER_H

#include <fstream>
#include <deque>
#include <array>
#include "behaviortree_cpp/loggers/abstract_logger.h"

#include "behaviortree_eut_plugins/flatbuffers/bt_flatbuffer_helper_eut.h"

namespace BT
{
class FileLogger : public StatusChangeLogger
{
public:
  FileLogger(const Tree& tree, const char* filename, uint16_t buffer_size = 10, const bool store_values = false, const bool clean_output = true);

  virtual ~FileLogger() override;

  virtual void callback(Duration timestamp, const TreeNode& node, NodeStatus prev_status,
                        NodeStatus status) override;

  virtual void flush() override;

private:
  void writeSerializedTransitionMaps(const SerializedTransitionMaps& serialized_trans_tmaps);
  std::ofstream file_os_;

  std::chrono::high_resolution_clock::time_point start_time;

  std::vector<SerializedTransition> buffer_;
  std::vector<SerializedTransitionMaps> transition_maps_buffer_;

  size_t buffer_max_size_;
  const bool store_values_;
  const bool clean_output_;
};

}   // namespace BT

#endif   // BT_FILE_LOGGER_H
