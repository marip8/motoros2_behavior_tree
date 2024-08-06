#include <motoros2_behavior_tree/motoros2_bt_nodes.h>

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

}  // namespace motoros2_behavior_tree

BTCPP_EXPORT void BT_RegisterRosNodeFromPlugin(BT::BehaviorTreeFactory& factory,
                                               const BT::RosNodeParams& params)
{
  factory.registerNodeType<motoros2_behavior_tree::StartTrajModeNode>("StartTrajMode", params);
  factory.registerNodeType<motoros2_behavior_tree::StartPointQueueModeNode>("StartPointQueueMode", params);
}
