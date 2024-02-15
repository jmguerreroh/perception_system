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

#include "perception_system/PeopleDetectionNode.hpp"

namespace perception_system
{

PeopleDetectionNode::PeopleDetectionNode()
: rclcpp_cascade_lifecycle::CascadeLifecycleNode("people_detection_node") {}

CallbackReturnT PeopleDetectionNode::on_configure(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(
    get_logger(), "[%s] Configuring from [%s] state...", get_name(),
    state.label().c_str());

  pub_ = this->create_publisher<yolov8_msgs::msg::DetectionArray>(
    "/perception_system/people_detection", 10);

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT PeopleDetectionNode::on_activate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(
    get_logger(), "[%s] Activating from [%s] state...", get_name(),
    state.label().c_str());

  sub_ = this->create_subscription<yolov8_msgs::msg::DetectionArray>(
    "/yolo/detections_3d", 10,
    [this](yolov8_msgs::msg::DetectionArray::ConstSharedPtr msg) {return this->callback(msg);});

  pub_->on_activate();

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT PeopleDetectionNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(
    get_logger(), "[%s] Deactivating from [%s] state...", get_name(),
    state.label().c_str());

  sub_ = nullptr;

  pub_->on_deactivate();

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT PeopleDetectionNode::on_cleanup(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(
    get_logger(), "[%s] Cleaning up from [%s] state...", get_name(),
    state.label().c_str());

  pub_.reset();

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT PeopleDetectionNode::on_shutdown(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(
    get_logger(), "[%s] Shutting down from [%s] state...", get_name(),
    state.label().c_str());

  pub_.reset();

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT PeopleDetectionNode::on_error(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "[%s] Erroring from [%s] state...", get_name(), state.label().c_str());

  return CallbackReturnT::SUCCESS;
}

void PeopleDetectionNode::callback(
  const yolov8_msgs::msg::DetectionArray::ConstSharedPtr & msg)
{
  yolov8_msgs::msg::DetectionArray people_array;

  for (auto detection : msg->detections) {
    if (detection.class_name == "person") {
      people_array.detections.push_back(detection);
    }
  }

  if (people_array.detections.size() > 0) {
    pub_->publish(people_array);
  }
}

}  // namespace perception_system
