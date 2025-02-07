#include <future>
#include "behaviortree_eut_plugins/loggers/bt_zmq_publisher.h"

#include "behaviortree_eut_plugins/eut_utils.h"

#include "3rdparty/cppzmq/zmq.hpp"
#include <boost/algorithm/string/replace.hpp>

namespace BT
{
//std::atomic<bool> PublisherZMQ::ref_count(false);

std::vector<StringView> splitString(const StringView& strToSplit, char delimeter, bool escape)
{
  std::vector<StringView> splitted_strings;
  splitted_strings.reserve(4);

  size_t cursor_pos = 0;
  size_t last_split_pos = 0;
  while (cursor_pos < strToSplit.size())
  {
    size_t new_pos = strToSplit.find_first_of(delimeter, cursor_pos);

    if (new_pos == std::string::npos)
    {
      new_pos = strToSplit.size();
    }

    bool skip = false;
    if(escape)
      skip = (new_pos < strToSplit.size() && new_pos > 0 //check if delimiter has to be escaped
          && strToSplit.at(new_pos-1) == '\\');
    
    if(!skip)
    {
      StringView sv = {&strToSplit.data()[last_split_pos], new_pos - last_split_pos};
      splitted_strings.push_back(sv);
      last_split_pos = new_pos + 1;
    }

    cursor_pos = new_pos + 1;
  }
  return splitted_strings;
}

struct PublisherZMQ::Pimpl
{
  Pimpl() : context(1), publisher(context, ZMQ_PUB), server(context, ZMQ_REP)
  {}

  zmq::context_t context;
  zmq::socket_t publisher;
  zmq::socket_t server;
};

PublisherZMQ::PublisherZMQ(BT::DebuggableTree& debugTree, unsigned max_msg_per_second,
                           unsigned publisher_port, unsigned server_port,
                           const bool clean_output) :
  StatusChangeLogger(debugTree.tree()->rootNode()),
  tree_(debugTree.tree()),
  min_time_between_msgs_(std::chrono::microseconds(1000 * 1000) / max_msg_per_second),
  send_pending_(false),
  zmq_(new Pimpl()),
  clean_output_(clean_output)
{
  setTimestampType(BT::TimestampType::relative);
  dfirst_timestamp_ = std::chrono::high_resolution_clock::now();

  dsubscribers_ = debugTree.instantiateDebugSubscribers(
      [this](TimePoint timestamp, const TreeNode& node,
                                  NodeAdvancedStatus prev, NodeAdvancedStatus status) 
      {
        this->debugCallback(timestamp - dfirst_timestamp_, node, prev, status);
      }
  );

  // auto visitor = [this, dsubscribeCallback](DebugTreeNode::Ptr dnode) 
  // {
  //   if(dnode) dsubscribers_.push_back(dnode->subscribeToDebugStatusChange(std::move(dsubscribeCallback)));
  // };

  // tree_->applyVisitor(visitor);
  /*
    bool expected = false;
    if (!ref_count.compare_exchange_strong(expected, true))
    {
      throw LogicError("Only one instance of PublisherZMQ shall be created");
    }
  */
  if (publisher_port == server_port)
  {
    throw LogicError("The TCP ports of the publisher and the server must be "
                     "different");
  }


  char str[100];

  sprintf(str, "tcp://*:%d", publisher_port);
  zmq_->publisher.bind(str);
  sprintf(str, "tcp://*:%d", server_port);
  zmq_->server.bind(str);

  int timeout_ms = 100;
  zmq_->server.set(zmq::sockopt::rcvtimeo, timeout_ms);

  active_server_ = true;

  thread_ = std::thread([this, &debugTree]() {
    zmq::message_t req;
    while (active_server_)
    {
      try
      {
        zmq::recv_result_t received = zmq_->server.recv(req);
        if (received)
        {
          size_t received_data = received.value();
          if(received_data == 0)//No req data, simple GET -> fetch Tree (as was the original behaviour)
          {
            flatbuffers::FlatBufferBuilder builder(1024);
            std::pair<bool, uint16_t> paused_uid{false, 0};
            if(debugTree.isPaused())
            {
              paused_uid.first = true;
              for(const auto& debug_wrapper : debugTree.debugWrappers())
                if(debug_wrapper.second->isPaused())
                {
                  paused_uid.second = debug_wrapper.second->node()->UID();
                }
            }
            CreateFlatbuffersBehaviorTree(builder, *(debugTree.tree()), debugTree.inDebugMode(), debugTree.bpNodesUID(), paused_uid);

            tree_buffer_.resize(builder.GetSize());
            memcpy(tree_buffer_.data(), builder.GetBufferPointer(), builder.GetSize());
            zmq::message_t reply(tree_buffer_.size());
            memcpy(reply.data(), tree_buffer_.data(), tree_buffer_.size());
            zmq_->server.send(reply, zmq::send_flags::none);
          }

          else if(received_data == 1)//Ping cmd to check life status
          {
              // std::cout << "Check life status req" << std::flush << std::endl;
              bool resp = true;
              zmq::message_t reply(sizeof(bool));
              memcpy(reply.data(), &resp, sizeof(bool));
              zmq_->server.send(reply, zmq::send_flags::none);
          }

          else if(debugTree.inDebugMode())//received a debug cmd
          {
            const char* req_data_raw = static_cast<const char*>(req.data());
            char* req_data_cleaned = new char[req.size()+1];
            strncpy(req_data_cleaned, req_data_raw, req.size());//in req.size you're size that there is the actual content length
            req_data_cleaned[req.size()] = '\0';
            // std::cout << "received raw '" << req_data_cleaned << "' req_size = " << req.size() << " strlen = " << strlen(req_data_cleaned) << std::flush << std::endl;
            
            DebugCmd cmd = extractDebugCmd(req_data_cleaned);
            // std::cout << "Command " << toStr(cmd) << std::endl << std::flush;
            
            std::shared_ptr<DebugPayload> body = extractDebugPayload(req_data_cleaned);
            // std::cout << "Payload " << toStr(body) << std::endl << std::flush;
            
            /**/
            bool resp = false;
            switch(cmd)
            {
              case DebugCmd::RESUME: 
                resp = debugTree.debugResume();
                break;
              
              case DebugCmd::PAUSE: 
                resp = debugTree.debugResume(DebuggableTree::ExecMode::DEBUG_STEP); // trick to make it pause at the next node to be ticked
                break;
              
              case DebugCmd::TICK_STEP: 
                resp = debugTree.debugResume(DebuggableTree::ExecMode::DEBUG_STEP);
                break;
                
              case DebugCmd::SETBP: 
                resp = debugTree.setBreakpoint(body->uid);
                break;
                
              case DebugCmd::RMPB: 
                resp = debugTree.rmBreakpoint(body->uid);
                break;

              case DebugCmd::SETPORT: 
                resp = debugTree.updPortValue(body);
                break;

              case DebugCmd::OVERRIDE_TICK:
                if(body) resp = debugTree.setOverrideTick(body) && debugTree.debugResume(DebuggableTree::ExecMode::DEBUG_STEP);
                break;
              
              case DebugCmd::SETENABLED:
              case DebugCmd::SETDISABLED: 
                if(body) resp = debugTree.setDisabled(body->uid, cmd == DebugCmd::SETDISABLED);
                break;

              default: 
                resp = false;
                break;
            }

            delete[] req_data_cleaned;

            /*Works as simple ack: SUCCESS/FAILURE*/
            zmq::message_t reply(sizeof(bool));
            memcpy(reply.data(), &resp, sizeof(bool));
            zmq_->server.send(reply, zmq::send_flags::none);
          }

          else
          {
            //received debug cmds, but tree is not in debug mode, so for now just discard them 
            std::cout << "Here received " << received_data << std::flush << std::endl;
          }
        }
      }
      catch (zmq::error_t& err)
      {
        if (err.num() == ETERM)
        {
          std::cout << "[PublisherZMQ] Server quitting." << std::endl;
        }
        std::cout << "[PublisherZMQ] just died. Exception " << err.what() << std::endl;
        active_server_ = false;
      }

      req.rebuild();//clean req message after processing
    }
  });

  createStatusBuffer();
}

PublisherZMQ::~PublisherZMQ()
{
  active_server_ = false;
  if (thread_.joinable())
  {
    thread_.join();
  }
  if (send_pending_)
  {
    send_condition_variable_.notify_all();
    send_future_.get();
  }
  flush();
  zmq_->context.shutdown();
  delete zmq_;
 // ref_count = false;
}

Expected<std::string> PublisherZMQ::extractValue(const std::vector<StringView>& req_parts, const std::string& key)
{
  for(const auto& p : req_parts)
  {
    std::string key_value_pair = convertFromString<std::string>(p);
    size_t key_pos = key_value_pair.find(key); 
    if(key_pos != std::string::npos)
      return key_value_pair.substr(key_pos+key.length()+1);
  }
  return nonstd::make_unexpected<std::string>("It was not possible to extract the value with key '" + key + "'");
}

DebugCmd PublisherZMQ::extractDebugCmd(const char* req)
{
  auto parts = splitString(req, ',');
  auto val_opt = extractValue(parts, "cmd");
  if(parts.size() > 0 && val_opt.has_value())
    return convertFromString<DebugCmd>(val_opt.value());
  else
    return DebugCmd::BAD_REQUEST;
}

std::shared_ptr<DebugPayload> PublisherZMQ::extractDebugPayload(const char* req)
{
  auto parts = splitString(req, ',', true);
  if(parts.size() > 1)
  {
    Expected<std::string> uid_s_opt  = extractValue(parts, "uid");
    if(uid_s_opt.has_value())
    {
      const int uid = std::stoi(uid_s_opt.value());
      
      // PortDebugPayload specific fields
      Expected<std::string> port_name_opt  = extractValue(parts, "port_name");
      Expected<std::string> port_value_opt = extractValue(parts, "port_value");
      
      // TickDebugPayload specific fields
      Expected<std::string> status_opt  = extractValue(parts, "status");
      Expected<std::string> skip_tick_opt = extractValue(parts, "skip_tick");
      
      if(port_name_opt.has_value() && port_value_opt.has_value()) // Is PortDebugPayload?
      {
        const std::string port_value_escaped = boost::replace_all_copy(port_value_opt.value(),  "\\,", ","); // removed escape characters for commas
        return std::make_shared<PortDebugPayload>(uid, port_name_opt.value(), port_value_escaped);
      }

      else if(status_opt.has_value() && skip_tick_opt.has_value())// Is TickDebugPayload?
      {
        try
        {
          const BT::NodeAdvancedStatus status = convertFromString<BT::NodeAdvancedStatus>(status_opt.value());
          const bool skip_tick = convertFromString<bool>(skip_tick_opt.value());
          return std::make_shared<TickDebugPayload>(uid, status, skip_tick);
        }
        catch(const RuntimeError&){/*string converter failed -> fallback to simple debug payload*/}
      }

      // Return simple payload with uid
      return std::make_shared<DebugPayload>(uid);

    }
  }

  return nullptr; // NO Payload
}

void PublisherZMQ::createStatusBuffer()
{
  status_buffer_.clear();
  applyRecursiveVisitor(tree_->rootNode(), [this](TreeNode* node) {
    size_t index = status_buffer_.size();
    status_buffer_.resize(index + 3);
    flatbuffers::WriteScalar<uint16_t>(&status_buffer_[index], node->UID());
    flatbuffers::WriteScalar<int8_t>(
        &status_buffer_[index + 2],
        static_cast<int8_t>(convertToFlatbuffers(node->status())));
  });
}

void PublisherZMQ::updateTransitionBuffer(SerializedTransition&& transition, SerializedTransitionMaps&& serialized_transition_maps, const bool send_updates_if_possible)
{
  {
    std::unique_lock<std::mutex> lock(mutex_);
    transition_buffer_.push_back(transition);
    transition_maps_buffer_.push_back(serialized_transition_maps);
  }

  if (send_updates_if_possible && !send_pending_.exchange(true))
  {
    send_future_ = std::async(std::launch::async, [this]() {
      std::unique_lock<std::mutex> lock(mutex_);
      const bool is_server_inactive = send_condition_variable_.wait_for(
          lock, min_time_between_msgs_, [this]() { return !active_server_; });
      lock.unlock();
      if (!is_server_inactive)
      {
        flush();
      }
    });
  }
}

void PublisherZMQ::debugCallback(Duration timestamp, const TreeNode& node, NodeAdvancedStatus prev_status,
                          NodeAdvancedStatus status)
{
  // std::cout << "DEBUG Callback for node " << node.registrationName() << ":" << node.name() << ":" <<  node.UID() << 
  //               " from " << static_cast<int>(status) << " to " << static_cast<int>(prev_status) << "\n";
  SerializedTransition transition =
      SerializeTransition(node.UID(), timestamp, prev_status, status);
  // don't send port value map when resetting a node that already completed its execution (success or failure), because you might retrieve updated values from the BB which are not used by the idle node
  SerializedTransitionMaps serialized_transition_maps = 
    SerializeTransitionMaps(node.UID(), timestamp, 
      (isStatusCompleted(toNodeStatus(prev_status)) && status == BT::NodeAdvancedStatus::IDLE)? (BT::PortsValueMap{}) : (getPortValuesMap(node, PortDirection::INPUT, clean_output_)), 
      (isStatusCompleted(toNodeStatus(prev_status)) && status == BT::NodeAdvancedStatus::IDLE)? (BT::PortsValueMap{}) : (getPortValuesMap(node, PortDirection::OUTPUT, clean_output_)), 
      /*node.getBlackboardValuesMap()*/{}); // TODO: review bb serialization protocol: for now avoid to send within the msg the entire BB dump: overkill and barely used on the other side

  updateTransitionBuffer(std::move(transition), std::move(serialized_transition_maps));
}

void PublisherZMQ::callback(Duration timestamp, const TreeNode& node,
                            NodeStatus prev_status, NodeStatus status)
{
  // std::cout << "Callback for node " << node.registrationName() << ":" << node.name() << ":" <<  node.UID() << 
  //               " from " << static_cast<int>(status) << " to " << static_cast<int>(prev_status) << "\n";
  SerializedTransition transition =
      SerializeTransition(node.UID(), timestamp, toNodeAdvancedStatus(prev_status), toNodeAdvancedStatus(status));
  // don't send port value map when resetting a node that already completed its execution (success or failure), because you might retrieve updated values from the BB which are not used by the idle node
  SerializedTransitionMaps serialized_transition_maps = 
    SerializeTransitionMaps(node.UID(), timestamp, 
      (isStatusCompleted(prev_status) && status == BT::NodeStatus::IDLE)? (BT::PortsValueMap{}) : (getPortValuesMap(node, PortDirection::INPUT, clean_output_)), 
      (isStatusCompleted(prev_status) && status == BT::NodeStatus::IDLE)? (BT::PortsValueMap{}) : (getPortValuesMap(node, PortDirection::OUTPUT, clean_output_)), 
      /*node.getBlackboardValuesMap()*/{}); // TODO: review bb serialization protocol: for now avoid to send within the msg the entire BB dump: overkill and barely used on the other side

  updateTransitionBuffer(std::move(transition), std::move(serialized_transition_maps));
  
}

void PublisherZMQ::flush()
{
  zmq::message_t message;
  {
    std::unique_lock<std::mutex> lock(mutex_);

    size_t serialized_maps_size = 0;
    std::vector<std::string> flatten_transition_maps_buffer_;
    flatten_transition_maps_buffer_.reserve(transition_maps_buffer_.size());
    for(const auto& tmap : transition_maps_buffer_)
    {
      flatten_transition_maps_buffer_.push_back(flattenSerializeTransitionMaps(tmap));
      serialized_maps_size += flatten_transition_maps_buffer_[flatten_transition_maps_buffer_.size()-1].length() * sizeof(char);
      // serialized_maps_size += 
      //   tmap[0].length()*sizeof(tmap[0].c_str()[0]) + // INPUT PORTS FLATTEN MAP
      //   tmap[1].length()*sizeof(tmap[1].c_str()[0]) + // OUTPUT PORTS FLATTEN MAP
      //   tmap[2].length()*sizeof(tmap[2].c_str()[0]); // BLACKBOARD FLATTEN MAP
    }

    const size_t msg_size = 3 * sizeof(uint32_t) + 
      status_buffer_.size() + 
      (transition_buffer_.size() * sizeof(uint8_t) * 12) + 
      serialized_maps_size;

    message.rebuild(msg_size);
    uint8_t* data_ptr = static_cast<uint8_t*>(message.data());

    // first 4 bytes are the size of the header
    flatbuffers::WriteScalar<uint32_t>(data_ptr,
                                       static_cast<uint32_t>(status_buffer_.size()));
    data_ptr += sizeof(uint32_t);
    // copy the header part
    memcpy(data_ptr, status_buffer_.data(), status_buffer_.size());
    data_ptr += status_buffer_.size();

    // first 4 bytes are the size of the transition buffer
    flatbuffers::WriteScalar<uint32_t>(data_ptr,
                                       static_cast<uint32_t>(transition_buffer_.size()));
    data_ptr += sizeof(uint32_t);

    size_t transitions_size = 0;

    for (auto& transition : transition_buffer_)
    {
      transitions_size += transition.size();
      memcpy(data_ptr, transition.data(), transition.size());
      data_ptr += transition.size();
    }
    transition_buffer_.clear();

    // other 4 bytes which are the size of the transition serialized maps buffer
    // std::cout << "Putting as size header for ALL tmaps " << serialized_maps_size << std::flush << std::endl;
    flatbuffers::WriteScalar<uint32_t>(data_ptr,
                                       static_cast<uint32_t>(serialized_maps_size));
    data_ptr += sizeof(uint32_t);

    // uint16_t i = 0;
    for (const auto& flatten_tmaps : flatten_transition_maps_buffer_)
    {
      size_t st = flatten_tmaps.length()*sizeof(flatten_tmaps.c_str()[0]);
      memcpy(data_ptr, flatten_tmaps.data(), st);
      data_ptr += st;

      // std::cout << "\n\nSending:\n" <<  flatten_tmaps.c_str() << "\n" << std::flush;
    }
    transition_maps_buffer_.clear();

    // std::cout << "[PublisherZMQ] sending msg of size " << message.size() << 
    //   " ( size info headers = " << 3 * sizeof(uint32_t) << 
    //  " header = " << status_buffer_.size() << ", " << 
    //   " transitions = " << transitions_size << ", " <<
    //   " serialized_maps = " << serialized_maps_size << ")" << 
    //   std::flush << std::endl;

    // other 4 bytes for the size of the serialized maps 
    createStatusBuffer();
  }
  try
  {
    zmq_->publisher.send(message, zmq::send_flags::none);
    // std::cout << "[PublisherZMQ] msg sent" << std::flush << std::endl;
  }
  catch (zmq::error_t& err)
  {
    if (err.num() == ETERM)
    {
      std::cout << "[PublisherZMQ] Publisher quitting." << std::endl;
    }
    std::cout << "[PublisherZMQ] just died. Exception " << err.what() << std::endl;
  }

  send_pending_ = false;
  // printf("%.3f zmq send\n", std::chrono::duration<double>( std::chrono::high_resolution_clock::now().time_since_epoch() ).count());
}
}   // namespace BT
