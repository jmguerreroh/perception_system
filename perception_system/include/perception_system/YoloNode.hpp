#pragma once

#include "rclcpp/rclcpp.hpp"

#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp_cascade_lifecycle/rclcpp_cascade_lifecycle.hpp"

#include "cascade_lifecycle_msgs/msg/activation.hpp"
#include "cascade_lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"

namespace perception_system
{

using CallbackReturnT =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class YoloNode : public rclcpp_cascade_lifecycle::CascadeLifecycleNode
{
public:
  YoloNode();
  CallbackReturnT on_configure(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_activate(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_cleanup(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_shutdown(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_error(const rclcpp_lifecycle::State & state);

  rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr client_yolo_debug_node_;
  rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr client_yolo_detect_3d_node_;
  rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr client_yolo_tracking_node_;
  rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr client_yolo_node_;
};

}  // namespace perception_system
