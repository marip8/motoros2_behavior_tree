#pragma once

#include <behaviortree_ros2/bt_service_node.hpp>
#include <behaviortree_ros2/bt_topic_sub_node.hpp>
#include <motoros2_interfaces/srv/write_single_io.hpp>
#include <motoros2_interfaces/srv/read_single_io.hpp>
#include <motoros2_interfaces/srv/start_traj_mode.hpp>
#include <motoros2_interfaces/srv/start_point_queue_mode.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace motoros2_behavior_tree
{
inline static const std::string ERROR_MESSAGE_KEY = "error_message";

template <typename T>
class RosServiceNode : public BT::RosServiceNode<T>
{
public:
  using BT::RosServiceNode<T>::RosServiceNode;

  inline BT::NodeStatus onFailure(BT::ServiceNodeErrorCode error) override
  {
    std::stringstream ss;
    ss << "Service '" << BT::RosServiceNode<T>::prev_service_name_ << "'";

    switch (error)
    {
      case BT::SERVICE_UNREACHABLE:
        ss << " is unreachable";
        break;
      case BT::SERVICE_TIMEOUT:
        ss << " timed out";
        break;
      case BT::INVALID_REQUEST:
        ss << " was sent an invalid request";
        break;
      case BT::SERVICE_ABORTED:
        ss << " was aborted";
        break;
      default:
        break;
    }

    this->config().blackboard->set(ERROR_MESSAGE_KEY, ss.str());

    return BT::NodeStatus::FAILURE;
  }
};

class StartTrajModeNode : public RosServiceNode<motoros2_interfaces::srv::StartTrajMode>
{
public:
  using RosServiceNode<motoros2_interfaces::srv::StartTrajMode>::providedPorts;
  using RosServiceNode<motoros2_interfaces::srv::StartTrajMode>::RosServiceNode;

  bool setRequest(typename Request::SharedPtr& request) override;
  BT::NodeStatus onResponseReceived(const typename Response::SharedPtr& response) override;
};

class StartPointQueueModeNode : public RosServiceNode<motoros2_interfaces::srv::StartPointQueueMode>
{
public:
  using RosServiceNode<motoros2_interfaces::srv::StartPointQueueMode>::providedPorts;
  using RosServiceNode<motoros2_interfaces::srv::StartPointQueueMode>::RosServiceNode;

  bool setRequest(typename Request::SharedPtr& request) override;
  BT::NodeStatus onResponseReceived(const typename Response::SharedPtr& response) override;
};

class WriteSingleIONode : public RosServiceNode<motoros2_interfaces::srv::WriteSingleIO>
{
public:
  inline static std::string ADDRESS_KEY = "address";
  inline static std::string VALUE_KEY = "value";
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({ BT::InputPort(ADDRESS_KEY), BT::InputPort(VALUE_KEY) });
  }
  using RosServiceNode<motoros2_interfaces::srv::WriteSingleIO>::RosServiceNode;

  bool setRequest(typename Request::SharedPtr& request) override;
  BT::NodeStatus onResponseReceived(const typename Response::SharedPtr& response) override;
};

class ReadSingleIONode : public RosServiceNode<motoros2_interfaces::srv::ReadSingleIO>
{
public:
  inline static std::string ADDRESS_KEY = "address";
  inline static std::string VALUE_KEY = "value";
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({ BT::InputPort(ADDRESS_KEY), BT::OutputPort(VALUE_KEY) });
  }
  using RosServiceNode<motoros2_interfaces::srv::ReadSingleIO>::RosServiceNode;

  bool setRequest(typename Request::SharedPtr& request) override;
  BT::NodeStatus onResponseReceived(const typename Response::SharedPtr& response) override;
};

class AddJointsToTrajectoryNode : public BT::RosTopicSubNode<sensor_msgs::msg::JointState>
{
public:
  inline static std::string TRAJECTORY_INPUT_PORT_KEY = "input";
  inline static std::string TRAJECTORY_OUTPUT_PORT_KEY = "output";
  inline static std::string CONTROLLER_JOINT_NAMES_PARAM = "controller_joint_names";
  inline static BT::PortsList providedPorts()
  {
    return providedBasicPorts({ BT::InputPort<trajectory_msgs::msg::JointTrajectory>(TRAJECTORY_INPUT_PORT_KEY),
                                BT::OutputPort<trajectory_msgs::msg::JointTrajectory>(TRAJECTORY_OUTPUT_PORT_KEY) });
  }
  using BT::RosTopicSubNode<sensor_msgs::msg::JointState>::RosTopicSubNode;

  BT::NodeStatus onTick(const typename sensor_msgs::msg::JointState::SharedPtr& last_msg) override;
};

}  // namespace motoros2_behavior_tree
