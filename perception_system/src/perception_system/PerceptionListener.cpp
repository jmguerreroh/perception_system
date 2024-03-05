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
#include "perception_system_interfaces/msg/detection_array.hpp"


#include "perception_system/PerceptionListener.hpp"


namespace perception_system
{

PerceptionListener::PerceptionListener(const rclcpp::NodeOptions & options)
: CascadeLifecycleNode("perception_listener", options)
{
  this->add_activation("perception_debug_node");
  // set_interest("bowl", true);
}

using CallbackReturnT =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturnT
PerceptionListener::on_configure(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(
    get_logger(), "[%s] Configuring from [%s] state...", get_name(),
    state.label().c_str());

  // this->declare_parameter("interest1", "");
  // this->get_parameter("interest1", interest1_);
  // this->declare_parameter("interest2", "");
  // this->get_parameter("interest2", interest2_);
  this->declare_parameter("time", 0.001);
  this->get_parameter("time", time_);
  last_update_ = rclcpp::Clock(RCL_STEADY_TIME).now();

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
PerceptionListener::on_activate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(
    get_logger(), "[%s] Activating from [%s] state...", get_name(),
    state.label().c_str());

  std::string topic_name = std::string(get_namespace()) + "/all_perceptions";
  percept_sub_ = this->create_subscription<perception_system_interfaces::msg::DetectionArray>(
    topic_name, 10,
    std::bind(&PerceptionListener::perception_callback, this, std::placeholders::_1));

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
PerceptionListener::on_deactivate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(
    get_logger(), "[%s] Deactivating from [%s] state...", get_name(),
    state.label().c_str());

  percept_sub_ = nullptr;

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
PerceptionListener::perception_callback(perception_system_interfaces::msg::DetectionArray::UniquePtr msg)
{
  // get hz
  auto start = rclcpp::Clock(RCL_STEADY_TIME).now();

  // auto message = std::move(msg);
  for (auto & detection : msg->detections)
  {
    // Get msg (std::move)
    // If it is in last_perceptions_ update, if not, create a new element in the vector
    auto det = perceptions_.find(detection.unique_id);
    if (det != perceptions_.end())
    {
      det->second.msg.score = detection.score;
      det->second.msg.center2d = detection.center2d;
      det->second.msg.bbox2d = detection.bbox2d;
      det->second.msg.center3d = detection.center3d;
      det->second.msg.bbox3d = detection.bbox3d;

      if (detection.person_data)
      {
        det->second.msg.color_person = detection.color_person;
        det->second.msg.skeleton2d = detection.skeleton2d;
        det->second.msg.skeleton3d = detection.skeleton3d;
        det->second.msg.pointing_direction = detection.pointing_direction;
      }

      if (detection.object_data)
      {
        det->second.msg.color_object = detection.color_object;
        det->second.msg.collision = detection.collision;
      }
      det->second.time  = rclcpp::Clock(RCL_STEADY_TIME).now();
    }
    else
    {
      perceptions_.insert(std::pair<std::string, PerceptionData>(detection.unique_id, {detection.type, detection, rclcpp::Clock(RCL_STEADY_TIME).now()}));
    }
  }

  // get diff and assign to hz_callback_
  auto end = rclcpp::Clock(RCL_STEADY_TIME).now();
  auto diff = end - start;
  hz_callback_ = diff.seconds();

  // // show the perceptions
  // for (auto & perception : perceptions_)
  // {
  //   if (perception.second.msg.person_data)
  //     RCLCPP_INFO(get_logger(), "Perception: %s, %ld", perception.second.msg.unique_id.c_str(), perception.second.msg.color_person);
  //   else
  //     RCLCPP_INFO(get_logger(), "Perception: %s, H:%d, S:%d, V:%d", perception.second.msg.unique_id.c_str(), perception.second.msg.color_object[0], perception.second.msg.color_object[1], perception.second.msg.color_object[2]);  
  // }

  // // show the interests
  // for (auto & interest : interests_)
  // {
  //   RCLCPP_INFO(get_logger(), "Interest: %s, %d", interest.first.c_str(), interest.second.status);
  // }
}

void
PerceptionListener::update(bool force)
{
  // Check if last_update_ is less than hz_callback, and return
  auto now_update = rclcpp::Clock(RCL_STEADY_TIME).now();
  auto diff_update = now_update - last_update_;
  if ((!force) && (diff_update.seconds() < hz_callback_))
  {
    return;
  }

  // std::cout << "Hz update: " << diff_update.seconds() << std::endl;

  // Check if last_perceptions_ elements are too old, and remove it
  // Check if interests are too old, and call set_interest( , false)
  std::vector<std::string> to_remove;
  for (auto & perception : perceptions_)
  {
    auto now = rclcpp::Clock(RCL_STEADY_TIME).now();
    auto diff = now - perception.second.time;
    if (diff.seconds() > time_)
    {
      to_remove.push_back(perception.first);
    }
  }

  for (auto & id : to_remove)
  {
    perceptions_.erase(id);
  }

  // for (auto & perception : perceptions_)
  // {
  //   if (perception.second.msg.person_data)
  //     RCLCPP_INFO(get_logger(), "Perception: %s, %ld", perception.second.msg.unique_id.c_str(), perception.second.msg.color_person);
  //   else
  //     RCLCPP_INFO(get_logger(), "Perception: %s, H:%d, S:%d, V:%d", perception.second.msg.unique_id.c_str(), perception.second.msg.color_object[0], perception.second.msg.color_object[1], perception.second.msg.color_object[2]);  
  // }

  last_update_ = rclcpp::Clock(RCL_STEADY_TIME).now();
}

void
PerceptionListener::set_interest(const std::string & id, bool status)
{
  // if status == true, add to interests_ and call add_activation
  // if status == false, remove to interests_ and call remove_activation
  //std::cout << "Setting interest: " << id << " to " << status << " in instance %p" << uniqueInstance_ << std::endl;
  if (status)
  {  
    if (interests_.find(id) == interests_.end())
    {
      rclcpp::Clock steady_clock(RCL_STEADY_TIME);
      interests_.insert(std::pair<std::string, PerceptionInterest>(id, {status, steady_clock.now()}));
      if(id.find("person") != std::string::npos) {
        this->add_activation("perception_people_detection_node");
      }
      else {
        this->add_activation("perception_objects_detection_node");
      }
      RCLCPP_INFO(get_logger(), "Added interest: %s, %d", id.c_str(), status);
    }
    else
    {
      rclcpp::Clock steady_clock(RCL_STEADY_TIME);
      interests_.find(id)->second.time = steady_clock.now();
    }
  }
  else
  {
    interests_.erase(id);
    if(id.find("person") != std::string::npos) {
      this->remove_activation("perception_people_detection_node");
    }
    else {
      this->remove_activation("perception_objects_detection_node");
    }
    RCLCPP_INFO(get_logger(), "Removed interest: %s, %d", id.c_str(), status);
  }

}

std::vector<perception_system_interfaces::msg::Detection>
PerceptionListener::get_by_id(const std::string & id)
{
  // Create a vector from perceptions_ whose id match
  std::vector<perception_system_interfaces::msg::Detection> result;
  for (auto & perception : perceptions_)
  {
    if (perception.first == id)
    {
      result.push_back(perception.second.msg);
    }
  }
  return result;

}

std::vector<perception_system_interfaces::msg::Detection>
PerceptionListener::get_by_type(const std::string & type)
{
  // set_interest(type, true);
  // Create a vector from perceptions_ whose type match
  std::vector<perception_system_interfaces::msg::Detection> result;
  for (auto & perception : perceptions_)
  {
    if (perception.second.type == type)
    {
      result.push_back(perception.second.msg);
    }
  }
  return result;
}

}  // namespace perception_system