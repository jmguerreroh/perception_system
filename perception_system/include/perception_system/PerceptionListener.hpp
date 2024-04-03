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

class PerceptionListener : public rclcpp_cascade_lifecycle::CascadeLifecycleNode
{
public:
  static std::shared_ptr<PerceptionListener> getInstance()
  {
    if (uniqueInstance_ == nullptr) {
      uniqueInstance_ = std::make_shared<PerceptionListener>();
    }
    return uniqueInstance_;
  }

  explicit PerceptionListener(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  virtual ~PerceptionListener() {}

  void update(double hz = 30);

  void set_interest(const std::string & type, bool status = true);
  std::vector<perception_system_interfaces::msg::Detection> get_by_id(const std::string & id);
  std::vector<perception_system_interfaces::msg::Detection> get_by_type(const std::string & type);
  void publicTFinterest();

  using CallbackReturnT =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturnT on_configure(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_activate(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_cleanup(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_shutdown(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_error(const rclcpp_lifecycle::State & state);

private:
  static std::shared_ptr<PerceptionListener> uniqueInstance_;

  rclcpp::Subscription<perception_system_interfaces::msg::DetectionArray>::SharedPtr percept_sub_;
  std::map<std::string, PerceptionInterest> interests_;
  std::map<std::string, PerceptionData> perceptions_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  void perception_callback(perception_system_interfaces::msg::DetectionArray::UniquePtr msg);
  int publicTF(const perception_system_interfaces::msg::Detection & detected_object);


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
