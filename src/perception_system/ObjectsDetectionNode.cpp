/*
  Copyright (c) 2024 José Miguel Guerrero Hernández

  Licensed under the Attribution-ShareAlike 4.0 International (CC BY-SA 4.0) License;
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

      https://creativecommons.org/licenses/by-sa/4.0/

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.
*/

#include "perception_system/ObjectsDetectionNode.hpp"

namespace perception_system
{

ObjectsDetectionNode::ObjectsDetectionNode()
: rclcpp_cascade_lifecycle::CascadeLifecycleNode("objects_detection_node")
{
  // Add the activation of the people detection node
  this->add_activation("yolo_node");
}

CallbackReturnT ObjectsDetectionNode::on_configure(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(
    get_logger(), "[%s] Configuring from [%s] state...", get_name(),
    state.label().c_str());

  this->declare_parameter("classes", "all");
  this->get_parameter("classes", classes_);
  this->declare_parameter("debug", false);
  this->get_parameter("debug", debug_);
  this->declare_parameter("target_frame", "head_front_camera_link");
  this->get_parameter("target_frame", frame_id_);

  pub_ = this->create_publisher<yolov8_msgs::msg::DetectionArray>(
    "/perception_system/objects_detection", 10);

  markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/perception_system/bb_objects", 10);

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT ObjectsDetectionNode::on_activate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(
    get_logger(), "[%s] Activating from [%s] state...", get_name(),
    state.label().c_str());

  sub_ = this->create_subscription<yolov8_msgs::msg::DetectionArray>(
    "/yolo/detections_3d", 10,
    [this](yolov8_msgs::msg::DetectionArray::ConstSharedPtr msg) {return this->callback(msg);});

  pub_->on_activate();
  markers_pub_->on_activate();

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT ObjectsDetectionNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(
    get_logger(), "[%s] Deactivating from [%s] state...", get_name(),
    state.label().c_str());

  sub_ = nullptr;

  pub_->on_deactivate();
  markers_pub_->on_deactivate();

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT ObjectsDetectionNode::on_cleanup(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(
    get_logger(), "[%s] Cleaning up from [%s] state...", get_name(),
    state.label().c_str());

  pub_.reset();
  markers_pub_.reset();

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT ObjectsDetectionNode::on_shutdown(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(
    get_logger(), "[%s] Shutting down from [%s] state...", get_name(),
    state.label().c_str());

  pub_.reset();
  markers_pub_.reset();

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT ObjectsDetectionNode::on_error(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "[%s] Erroring from [%s] state...", get_name(), state.label().c_str());

  return CallbackReturnT::SUCCESS;
}

void ObjectsDetectionNode::callback(
  const yolov8_msgs::msg::DetectionArray::ConstSharedPtr & msg)
{
  yolov8_msgs::msg::DetectionArray objects_array;
  visualization_msgs::msg::MarkerArray marker_array;

  for (auto detection : msg->detections) {
    if ((classes_.find("all") != std::string::npos) ||
      (classes_.find(detection.class_name) != std::string::npos))
    {
      objects_array.detections.push_back(detection);

      if (debug_) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = frame_id_;
        marker.header.stamp = msg->header.stamp;
        marker.ns = "perception_system";
        marker.id = stoi(detection.id);
        marker.text = detection.class_name;
        marker.frame_locked = false;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = detection.bbox3d.center.position.x;
        marker.pose.position.y = detection.bbox3d.center.position.y;
        marker.pose.position.z = detection.bbox3d.center.position.z;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = detection.bbox3d.size.x;
        marker.scale.y = detection.bbox3d.size.y;
        marker.scale.z = detection.bbox3d.size.z;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.lifetime = rclcpp::Duration::from_seconds(0.5);
        marker_array.markers.push_back(marker);
      }
    }
  }

  if (objects_array.detections.size() > 0) {
    pub_->publish(objects_array);
    if (debug_) {
      markers_pub_->publish(marker_array);
    }
  }
}

}  // namespace perception_system
