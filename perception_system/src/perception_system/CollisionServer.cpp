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

#include <memory>

#include "perception_system/CollisionServer.hpp"


using namespace std::chrono_literals;
using namespace std::placeholders;

namespace perception_system
{

CollisionServer::CollisionServer(const rclcpp::NodeOptions & options)
: rclcpp_cascade_lifecycle::CascadeLifecycleNode("collision_server_node", options)
{
  // todo: add dependencies 

  // todo: declare params
}
CollisionServer::~CollisionServer()
{
}

CallbackReturn
CollisionServer::on_configure(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(
    get_logger(), "[%s] Configuring from [%s] state...", get_name(),
    state.label().c_str());

  last_pc_ = nullptr; 
  collison_service_ = create_service<perception_system_interfaces::srv::ExtractCollision>(
    "extract_collision", std::bind(&CollisionServer::collision_service_callback, this, _1, _2));

  point_cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "pointcloud_in",1,
    std::bind(&CollisionServer::pc_callback, this, std::placeholders::_1));
  // latched topic  
  point_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    "pointcloud_filtered", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  return CallbackReturn::SUCCESS;
}

CallbackReturn
CollisionServer::on_activate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(
    get_logger(), "[%s] Activating from [%s] state...", get_name(),
    state.label().c_str());

  point_cloud_pub_->on_activate();
  return CallbackReturn::SUCCESS;
}

CallbackReturn
CollisionServer::on_deactivate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(
  get_logger(), "[%s] Deactivating from [%s] state...", get_name(),
  state.label().c_str());

  point_cloud_pub_->on_deactivate();

  return CallbackReturn::SUCCESS;
}

CallbackReturn
CollisionServer::on_cleanup(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(
  get_logger(), "[%s] Cleanning up from [%s] state...", get_name(),
  state.label().c_str());

  point_cloud_pub_.reset();
  collison_service_.reset();
  collison_service_.reset();
  return CallbackReturn::SUCCESS;
}

CallbackReturn
CollisionServer::on_shutdown(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(
    get_logger(), "[%s] Shutting down from [%s] state...", get_name(),
    state.label().c_str());

  return CallbackReturn::SUCCESS;
}

CallbackReturn
CollisionServer::on_error(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "[%s] Erroring from [%s] state...", get_name(), state.label().c_str());

  return CallbackReturn::SUCCESS;
}

void CollisionServer::pc_callback(sensor_msgs::msg::PointCloud2::UniquePtr msg)
{
  RCLCPP_INFO(get_logger(), "Received point cloud");
  last_pc_ = std::move(msg);
}

void CollisionServer::collision_service_callback(
  const std::shared_ptr<perception_system_interfaces::srv::ExtractCollision::Request> request,
  std::shared_ptr<perception_system_interfaces::srv::ExtractCollision::Response> response)
{
  if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    RCLCPP_WARN(
      get_logger(),
      "Received request but not in ACTIVE state, ignoring!");
  }
    
  if (last_pc_ == nullptr) {  
    response->success = false;
    RCLCPP_ERROR(
      get_logger(),
      "No point cloud received, cannot process collision extraction");    
    return;
  }
  pcl::PointCloud<pcl::PointXYZRGB> pointcloud;
  pcl::fromROSMsg(*last_pc_, pointcloud);

  auto plane = ExtractNPlanes(pointcloud, 0);
  pcl::toROSMsg(plane, filtered_pc_);

  response->collision_points = filtered_pc_;  

  response->success = true;

  point_cloud_pub_->publish(filtered_pc_);
}

}  // namespace perception_system
