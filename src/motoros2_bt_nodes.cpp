#include <motoros2_behavior_tree/motoros2_bt_nodes.h>

template <typename T>
T getBTInput(const BT::TreeNode* node, const std::string& port)
{
  BT::Expected<T> input = node->getInput<T>(port);
  if (!input)
    throw BT::RuntimeError("Failed to get required input value: '" + input.error() + "'");

  return input.value();
}

namespace motoros2_behavior_tree
{
bool StartTrajModeNode::setRequest(typename Request::SharedPtr& /*request*/)
{
  return true;
}

BT::NodeStatus StartTrajModeNode::onResponseReceived(const typename Response::SharedPtr& response)
{
  switch(response->result_code.value)
  {
  case motoros2_interfaces::msg::MotionReadyEnum::READY:
    return BT::NodeStatus::SUCCESS;
  default:
    config().blackboard->set(ERROR_MESSAGE_KEY, response->message);
    return BT::NodeStatus::FAILURE;
  }
}

bool StartPointQueueModeNode::setRequest(typename Request::SharedPtr& /*request*/)
{
  return true;
}

BT::NodeStatus StartPointQueueModeNode::onResponseReceived(const typename Response::SharedPtr& response)
{
  switch(response->result_code.value)
  {
  case motoros2_interfaces::msg::MotionReadyEnum::READY:
    return BT::NodeStatus::SUCCESS;
  default:
    config().blackboard->set(ERROR_MESSAGE_KEY, response->message);
    return BT::NodeStatus::FAILURE;
  }
}

bool WriteSingleIONode::setRequest(typename Request::SharedPtr& request)
{
  request->address = getBTInput<uint32_t>(this, ADDRESS_KEY);
  request->value = getBTInput<uint32_t>(this, VALUE_KEY);
  return true;
}

BT::NodeStatus WriteSingleIONode::onResponseReceived(const typename Response::SharedPtr& response)
{
  if (!response->success)
  {
    config().blackboard->set(ERROR_MESSAGE_KEY, response->message);
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::SUCCESS;
}

bool ReadSingleIONode::setRequest(typename Request::SharedPtr& request)
{
  request->address = getBTInput<uint32_t>(this, ADDRESS_KEY);
  return true;
}

BT::NodeStatus ReadSingleIONode::onResponseReceived(const typename Response::SharedPtr& response)
{
  if (!response->success)
  {
    config().blackboard->set(ERROR_MESSAGE_KEY, response->message);
    return BT::NodeStatus::FAILURE;
  }
  setOutput(VALUE_KEY, response->value);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace motoros2_behavior_tree

BTCPP_EXPORT void BT_RegisterRosNodeFromPlugin(BT::BehaviorTreeFactory& factory,
                                               const BT::RosNodeParams& params)
{
  factory.registerNodeType<motoros2_behavior_tree::StartTrajModeNode>("StartTrajMode", params);
  factory.registerNodeType<motoros2_behavior_tree::StartPointQueueModeNode>("StartPointQueueMode", params);
  factory.registerNodeType<motoros2_behavior_tree::WriteSingleIONode>("WriteSingleIO", params);
  factory.registerNodeType<motoros2_behavior_tree::ReadSingleIONode>("ReadSingleIO", params);
}
