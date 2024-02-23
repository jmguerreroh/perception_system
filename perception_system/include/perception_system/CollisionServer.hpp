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

#ifndef COLLISION_SERVER_HPP_
#define COLLISION_SERVER_HPP_

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "perception_system_interfaces/srv/extract_collision.hpp"

#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp_cascade_lifecycle/rclcpp_cascade_lifecycle.hpp"

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>

#include "perception_system/PerceptionFunctions.hpp"

namespace perception_system
{

using CallbackReturn =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class CollisionServer : public rclcpp_cascade_lifecycle::CascadeLifecycleNode
{
public:
  CollisionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~CollisionServer();

  CallbackReturn on_configure(const rclcpp_lifecycle::State & state);
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state);
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state);
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state);
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state);
  CallbackReturn on_error(const rclcpp_lifecycle::State & state);

private:
  void pc_callback(sensor_msgs::msg::PointCloud2::UniquePtr msg);
  void collision_service_callback(
    const std::shared_ptr<perception_system_interfaces::srv::ExtractCollision::Request> request,
    std::shared_ptr<perception_system_interfaces::srv::ExtractCollision::Response> response);

  rclcpp::Service<perception_system_interfaces::srv::ExtractCollision>::SharedPtr collison_service_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;

  sensor_msgs::msg::PointCloud2::UniquePtr last_pc_;
  sensor_msgs::msg::PointCloud2 filtered_pc_;

};

}  // namespace perception_system

#endif  // COLLISION_SERVER_HPP_
