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

#ifndef PERCEPTION_SYSTEM__PERCEPTION_LISTENER_HPP_
#define PERCEPTION_SYSTEM__PERCEPTION_LISTENER_HPP_

#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_cascade_lifecycle/rclcpp_cascade_lifecycle.hpp"

#include "perception_system_interfaces/msg/detection.hpp"
#include "perception_system_interfaces/msg/detection_array.hpp"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

namespace perception_system
{

struct PerceptionData
{
  std::string type;
  perception_system_interfaces::msg::Detection msg;
  rclcpp::Time time;
};

struct PerceptionInterest
{
  bool status;
  rclcpp::Time time;
};

class PerceptionListener
{
public:
  static std::shared_ptr<PerceptionListener> getInstance(
    std::shared_ptr<rclcpp_cascade_lifecycle::CascadeLifecycleNode> parent_node)
  {
    if (uniqueInstance_ == nullptr) {
      uniqueInstance_ = std::make_shared<PerceptionListener>(parent_node);
    }
    return uniqueInstance_;
  }

  explicit PerceptionListener(std::shared_ptr<rclcpp_cascade_lifecycle::CascadeLifecycleNode> parent_node);

  virtual ~PerceptionListener() {}

  void update(double hz = 30);

  void set_interest(const std::string & type, bool status = true);
  std::vector<perception_system_interfaces::msg::Detection> get_by_id(const std::string & id);
  std::vector<perception_system_interfaces::msg::Detection> get_by_type(const std::string & type);
  // directly publish the TF
  int publicTF(
    const perception_system_interfaces::msg::Detection & detected_object,
    const std::string & custom_suffix = "");
  void publicTFinterest();
  // public tfs with custom sorting
  void publicSortedTFinterest(
    std::function<bool(const perception_system_interfaces::msg::Detection &,
    const perception_system_interfaces::msg::Detection &)> comp = [] (const perception_system_interfaces::msg::Detection & a, const perception_system_interfaces::msg::Detection & b) {
      // Default sorting behavior
      return a.center3d.position.z < b.center3d.position.z;
    });

private:
  static std::shared_ptr<PerceptionListener> uniqueInstance_;
  std::shared_ptr<rclcpp_cascade_lifecycle::CascadeLifecycleNode> parent_node_;

  rclcpp::Subscription<perception_system_interfaces::msg::DetectionArray>::SharedPtr percept_sub_;
  std::map<std::string, PerceptionInterest> interests_;
  std::map<std::string, PerceptionData> perceptions_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  perception_system_interfaces::msg::DetectionArray::UniquePtr last_msg_;

  void perception_callback(perception_system_interfaces::msg::DetectionArray::UniquePtr msg);


  double max_time_perception_;
  double max_time_interest_;
  rclcpp::Time last_update_;

  std::string tf_frame_camera_;
  std::string tf_frame_map_;
};

}  // namespace perception_system

std::shared_ptr<perception_system::PerceptionListener> perception_system::PerceptionListener::
uniqueInstance_;


#endif  // PERCEPTION_SYSTEM__PERCEPTION_LISTENER_HPP_
