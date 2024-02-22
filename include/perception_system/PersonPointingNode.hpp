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

#ifndef PERSON_POINTING_NODE_HPP_
#define PERSON_POINTING_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "yolov8_msgs/msg/detection_array.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "std_msgs/msg/u_int8.hpp"

#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp_cascade_lifecycle/rclcpp_cascade_lifecycle.hpp"

#include "cascade_lifecycle_msgs/msg/activation.hpp"
#include "cascade_lifecycle_msgs/msg/state.hpp"

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include "perception_system/PerceptionFunctions.hpp"

namespace perception_system
{

using CallbackReturnT =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class PersonPointingNode : public rclcpp_cascade_lifecycle::CascadeLifecycleNode
{
public:
  PersonPointingNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  CallbackReturnT on_configure(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_activate(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_cleanup(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_shutdown(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_error(const rclcpp_lifecycle::State & state);

private:
  void callback(const yolov8_msgs::msg::DetectionArray::ConstSharedPtr & msg);

  rclcpp::Subscription<yolov8_msgs::msg::DetectionArray>::SharedPtr sub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::UInt8>::SharedPtr pub_;

  bool debug_;
  int64_t unique_id_;
};

}  // namespace perception_system

#endif  // PERSON_POINTING_NODE_HPP_
