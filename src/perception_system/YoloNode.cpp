#include "perception_system/YoloNode.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"

namespace perception_system
{

YoloNode::YoloNode()
: rclcpp_cascade_lifecycle::CascadeLifecycleNode("yolo_node")
{
  // Add the activation of YOLO nodes
  this->add_activation("yolov8_node");
  this->add_activation("yolov8_tracking_node");
  this->add_activation("yolov8_detect_3d_node");
  this->add_activation("yolov8_debug_node");

  // Crear un cliente de servicio para el servicio de cambio de estado
  client_yolo_debug_node_ = this->create_client<lifecycle_msgs::srv::ChangeState>(
    "/yolo/yolov8_debug_node/change_state");
  client_yolo_detect_3d_node_ = this->create_client<lifecycle_msgs::srv::ChangeState>(
    "/yolo/yolov8_detect_3d_node/change_state");
  client_yolo_tracking_node_ = this->create_client<lifecycle_msgs::srv::ChangeState>(
    "/yolo/yolov8_tracking_node/change_state");
  client_yolo_node_ = this->create_client<lifecycle_msgs::srv::ChangeState>(
    "/yolo/yolov8_node/change_state");


  // // Esperar a que el servicio esté disponible
  while (!client_yolo_debug_node_->wait_for_service(std::chrono::seconds(1) ) &&
    !client_yolo_detect_3d_node_->wait_for_service(std::chrono::seconds(1) ) &&
    !client_yolo_tracking_node_->wait_for_service(std::chrono::seconds(1) ) &&
    !client_yolo_node_->wait_for_service(std::chrono::seconds(1) ) )
  {
    RCLCPP_INFO(get_logger(), "Waiting for the change state service...");
  }

  RCLCPP_INFO(get_logger(), "Service is available in YOLO nodes");
}

CallbackReturnT YoloNode::on_configure(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(
    get_logger(), "[%s] Configuring from [%s] state...", get_name(),
    state.label().c_str());

  // Create a request to change to the desired state
  auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  request->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE;

  // Call the service to change the state
  client_yolo_debug_node_->async_send_request(request);
  client_yolo_detect_3d_node_->async_send_request(request);
  client_yolo_tracking_node_->async_send_request(request);
  client_yolo_node_->async_send_request(request);

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT YoloNode::on_activate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(
    get_logger(), "[%s] Activating from [%s] state...", get_name(),
    state.label().c_str());

  // Create a request to change to the desired state
  auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  request->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE;

  // Call the service to change the state
  client_yolo_debug_node_->async_send_request(request);
  client_yolo_detect_3d_node_->async_send_request(request);
  client_yolo_tracking_node_->async_send_request(request);
  client_yolo_node_->async_send_request(request);

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT YoloNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(
    get_logger(), "[%s] Deactivating from [%s] state...", get_name(),
    state.label().c_str());

  // Create a request to change to the desired state
  auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  request->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE;

  // Call the service to change the state
  client_yolo_debug_node_->async_send_request(request);
  client_yolo_detect_3d_node_->async_send_request(request);
  client_yolo_tracking_node_->async_send_request(request);
  client_yolo_node_->async_send_request(request);

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT YoloNode::on_cleanup(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(
    get_logger(), "[%s] Cleaning up from [%s] state...", get_name(),
    state.label().c_str());

  // Create a request to change to the desired state
  auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  request->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP;

  // Call the service to change the state
  client_yolo_debug_node_->async_send_request(request);
  client_yolo_detect_3d_node_->async_send_request(request);
  client_yolo_tracking_node_->async_send_request(request);
  client_yolo_node_->async_send_request(request);


  return CallbackReturnT::SUCCESS;
}

CallbackReturnT YoloNode::on_shutdown(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(
    get_logger(), "[%s] Shutting down from [%s] state...", get_name(),
    state.label().c_str());

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT YoloNode::on_error(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "[%s] Erroring from [%s] state...", get_name(), state.label().c_str());

  return CallbackReturnT::SUCCESS;
}

}  // namespace perception_system
