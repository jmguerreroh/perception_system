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

#include "perception/FollowPersonNode.hpp"

namespace perception
{

FollowPersonNode::FollowPersonNode()
: rclcpp_lifecycle::LifecycleNode("follow_person_node") {}

CallbackReturnT FollowPersonNode::on_configure(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(
    get_logger(), "[%s] Configuring from [%s] state...", get_name(),
    state.label().c_str());

  this->declare_parameter("target_frame", "head_front_camera_link");
  this->get_parameter("target_frame", frame_id_);
  this->declare_parameter("debug", false);
  this->get_parameter("debug", debug_);

  markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/perception/bb_objects", 10);

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT FollowPersonNode::on_activate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(
    get_logger(), "[%s] Activating from [%s] state...", get_name(),
    state.label().c_str());

  sub_ = this->create_subscription<yolov8_msgs::msg::DetectionArray>(
    "/perception/people_detection", 10,
    [this](yolov8_msgs::msg::DetectionArray::ConstSharedPtr msg) {return this->callback(msg);});

  tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

  markers_pub_->on_activate();

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT FollowPersonNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(
    get_logger(), "[%s] Deactivating from [%s] state...", get_name(),
    state.label().c_str());

  sub_ = nullptr;
  tf_broadcaster_.reset();
  markers_pub_->on_deactivate();

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT FollowPersonNode::on_cleanup(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(
    get_logger(), "[%s] Cleaning up from [%s] state...", get_name(),
    state.label().c_str());

  tf_broadcaster_.reset();
  markers_pub_.reset();

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT FollowPersonNode::on_shutdown(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(
    get_logger(), "[%s] Shutting down from [%s] state...", get_name(),
    state.label().c_str());

  tf_broadcaster_.reset();
  markers_pub_.reset();

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT FollowPersonNode::on_error(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "[%s] Erroring from [%s] state...", get_name(), state.label().c_str());

  return CallbackReturnT::SUCCESS;
}

double distance3D(double x1, double y1, double z1, double x2, double y2, double z2)
{
  return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2) + std::pow(z2 - z1, 2));
}

void FollowPersonNode::callback(
  const yolov8_msgs::msg::DetectionArray::ConstSharedPtr & msg)
{
  // Find the closest person
  auto global_detection = msg->detections[0];
  auto global_dist = distance3D(
    global_detection.bbox3d.center.position.x,
    global_detection.bbox3d.center.position.y,
    global_detection.bbox3d.center.position.z, 0, 0, 0);

  for (auto detection : msg->detections) {
    auto dist_min = distance3D(
      detection.bbox3d.center.position.x,
      detection.bbox3d.center.position.y,
      detection.bbox3d.center.position.z, 0, 0, 0);

    if (dist_min < global_dist) {
      global_detection = detection;
      global_dist = dist_min;
    }
  }

  // Public transform
  static_transform_stamped_.header.stamp = msg->header.stamp;
  static_transform_stamped_.header.frame_id = frame_id_;
  static_transform_stamped_.child_frame_id = "person";
  static_transform_stamped_.transform.translation.x = global_detection.bbox3d.center.position.x;
  static_transform_stamped_.transform.translation.y = global_detection.bbox3d.center.position.y;
  static_transform_stamped_.transform.translation.z = global_detection.bbox3d.center.position.z;
  static_transform_stamped_.transform.rotation.w = 1.0;

  tf_broadcaster_->sendTransform(static_transform_stamped_);

  if (debug_) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id_;
    marker.header.stamp = msg->header.stamp;
    marker.ns = "perception";
    marker.id = 0;
    marker.text = "person";
    marker.frame_locked = false;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = global_detection.bbox3d.center.position.x;
    marker.pose.position.y = global_detection.bbox3d.center.position.y;
    marker.pose.position.z = global_detection.bbox3d.center.position.z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = global_detection.bbox3d.size.x;
    marker.scale.y = global_detection.bbox3d.size.y;
    marker.scale.z = global_detection.bbox3d.size.z;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    visualization_msgs::msg::MarkerArray marker_array;
    marker_array.markers.push_back(marker);
    markers_pub_->publish(marker_array);
  }
}

}  // namespace perception
