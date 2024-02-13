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

#ifndef FOLLOW_PERSON_NODE_HPP_
#define FOLLOW_PERSON_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "yolov8_msgs/msg/detection_array.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>

#include "perception_system/PerceptionFunctions.hpp"

namespace perception_system
{

using CallbackReturnT =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class FollowPersonNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  FollowPersonNode();

  CallbackReturnT on_configure(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_activate(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_cleanup(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_shutdown(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_error(const rclcpp_lifecycle::State & state);

private:
  void callback(const yolov8_msgs::msg::DetectionArray::ConstSharedPtr & msg);

  rclcpp::Subscription<yolov8_msgs::msg::DetectionArray>::SharedPtr sub_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
  geometry_msgs::msg::TransformStamped static_transform_stamped_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;

  std::string frame_id_;
  bool debug_;
  int64_t unique_id_;
};

}  // namespace perception_system

#endif  // FOLLOW_PERSON_NODE_HPP_
