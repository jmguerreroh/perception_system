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


#include "rclcpp/rclcpp.hpp"
#include "rclcpp_cascade_lifecycle/rclcpp_cascade_lifecycle.hpp"

#include "perception_system_interfaces/msg/detection.hpp"


#include "perception_system/PerceptionListener.hpp"


namespace perception_system
{

PerceptionListener::PerceptionListener(const rclcpp::NodeOptions & options)
: CascadeLifecycleNode("perception_listener", options)
{

}

using CallbackReturnT =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturnT
PerceptionListener::on_configure(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(
    get_logger(), "[%s] Configuring from [%s] state...", get_name(),
    state.label().c_str());

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
PerceptionListener::on_activate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(
    get_logger(), "[%s] Activating from [%s] state...", get_name(),
    state.label().c_str());

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
PerceptionListener::on_deactivate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(
    get_logger(), "[%s] Deactivating from [%s] state...", get_name(),
    state.label().c_str());


  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
PerceptionListener::on_cleanup(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(
    get_logger(), "[%s] Cleaning up from [%s] state...", get_name(),
    state.label().c_str());

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
PerceptionListener::on_shutdown(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(
    get_logger(), "[%s] Shutting down from [%s] state...", get_name(),
    state.label().c_str());


  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
PerceptionListener::on_error(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "[%s] Erroring from [%s] state...", get_name(), state.label().c_str());

  return CallbackReturnT::SUCCESS;
}

void
PerceptionListener::perception_callback(perception_system_interfaces::msg::Detection::UniquePtr msg)
{
  // Get msg (std::move)
  // If it is in last_perceptions_ update, if not, create a new element in the vector
}

void
PerceptionListener::update()
{
  // Check if last_perceptions_ elements are too old, and remove it
  // Check if interests are too old, and call set_interest( , false)
}

void
PerceptionListener::set_interest(const std::string & type, bool status)
{
  // if status == true, add to interests_ and call add_activation
  // if status == false, remove to interests_ and call remove_activation
}

std::vector<perception_system_interfaces::msg::Detection>
PerceptionListener::get_by_id(const std::string & id)
{
  // Create a vector from last_perceptions_ whose id match
}

std::vector<perception_system_interfaces::msg::Detection>
PerceptionListener::get_by_type(const std::string & type)
{
  // Create a vector from last_perceptions_ whose type match
}

}  // namespace perception_system
