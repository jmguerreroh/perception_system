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

#include "perception_system/ObjectsDetectionNode.hpp"

namespace perception_system
{

ObjectsDetectionNode::ObjectsDetectionNode(const rclcpp::NodeOptions & options)
: rclcpp_cascade_lifecycle::CascadeLifecycleNode("perception_objects_detection", "perception_system", options)
{
  // Declare the parameters
  this->declare_parameter("classes", "all");
  this->declare_parameter("target_frame", "head_front_camera_link");
  this->declare_parameter("debug", false);

  // Add the activation of the objects detection node
  this->add_activation("yolo_node");
  this->add_activation("detect_3d_node");
  this->add_activation("tracking_node");
}

CallbackReturnT ObjectsDetectionNode::on_configure(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(
    get_logger(), "[%s] Configuring from [%s] state...", get_name(),
    state.label().c_str());

  this->get_parameter("classes", classes_);
  this->get_parameter("target_frame", frame_id_);
  if (this->get_parameter("debug").as_bool()) {
    this->add_activation("debug_node");
  }

  pub_ = this->create_publisher<perception_system_interfaces::msg::DetectionArray>(
    "all_perceptions", 10);

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT ObjectsDetectionNode::on_activate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(
    get_logger(), "[%s] Activating from [%s] state...", get_name(),
    state.label().c_str());

  std::string topic_name = "detections_3d";
  sub_ = this->create_subscription<yolo_msgs::msg::DetectionArray>(
    topic_name, 10,
    [this](yolo_msgs::msg::DetectionArray::ConstSharedPtr msg) {return this->callback(msg);});

  pub_->on_activate();

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT ObjectsDetectionNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(
    get_logger(), "[%s] Deactivating from [%s] state...", get_name(),
    state.label().c_str());

  sub_ = nullptr;

  pub_->on_deactivate();

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT ObjectsDetectionNode::on_cleanup(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(
    get_logger(), "[%s] Cleaning up from [%s] state...", get_name(),
    state.label().c_str());

  pub_.reset();

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT ObjectsDetectionNode::on_shutdown(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(
    get_logger(), "[%s] Shutting down from [%s] state...", get_name(),
    state.label().c_str());

  pub_.reset();

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT ObjectsDetectionNode::on_error(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "[%s] Erroring from [%s] state...", get_name(), state.label().c_str());

  return CallbackReturnT::SUCCESS;
}

void ObjectsDetectionNode::callback(
  const yolo_msgs::msg::DetectionArray::ConstSharedPtr & msg)
{
  // Convert from sensor_msgs::Image to cv::Mat using cv_bridge
  cv_bridge::CvImagePtr image_rgb_ptr;
  try {
    image_rgb_ptr = cv_bridge::toCvCopy(msg->source_img, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }
  cv::Mat image = image_rgb_ptr->image;

  perception_system_interfaces::msg::DetectionArray perception_array;
  perception_array.header = msg->header;
  perception_array.source_img = msg->source_img;

  for (auto detection : msg->detections) {
    if ((detection.class_name != "person") && ((classes_.find("all") != std::string::npos) ||
      (classes_.find(detection.class_name) != std::string::npos)))
    {
      perception_system_interfaces::msg::Detection perception;
      perception.header = msg->header;
      std::string id = underscore(detection.class_name + "_" + detection.id);
      perception.header.frame_id = frame_id_;
      perception.unique_id = id;
      perception.type = detection.class_name;
      perception.class_id = stoi(detection.id);
      perception.class_name = detection.class_name;
      perception.score = detection.score;
      perception.center2d.x = detection.bbox.center.position.x;
      perception.center2d.y = detection.bbox.center.position.y;
      perception.bbox2d.x = detection.bbox.size.x;
      perception.bbox2d.y = detection.bbox.size.y;
      perception.bbox2d.z = 0.0;
      perception.center3d = detection.bbox3d.center;
      perception.bbox3d = detection.bbox3d.size;

      cv::Point2d min_pt = cv::Point2d(
        round(detection.bbox.center.position.x - detection.bbox.size.x / 2.0),
        round(detection.bbox.center.position.y - detection.bbox.size.y / 2.0));
      cv::Point2d max_pt = cv::Point2d(
        round(detection.bbox.center.position.x + detection.bbox.size.x / 2.0),
        round(detection.bbox.center.position.y + detection.bbox.size.y / 2.0));

      min_pt = checkPoint(min_pt, image.size());
      max_pt = checkPoint(max_pt, image.size());

      cv::Mat roi = image(cv::Rect(min_pt, max_pt));
      sensor_msgs::msg::Image::SharedPtr img_msg;
      try {
        // Convert from cv::Mat to sensor_msgs::msg::Image
        img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", roi).toImageMsg();
      } catch (cv_bridge::Exception & e) {
        RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
        return;
      }
      perception.image = *img_msg;
      // Convert the ROI to the HSV color space
      cv::Mat hsvRoi;
      cv::cvtColor(roi, hsvRoi, cv::COLOR_BGR2HSV);
      // Depends on the object
      perception.collision = false;
      // color object
      std::vector<int8_t> avg = mean_color(hsvRoi);
      perception.color_object = avg;

      perception_array.detections.push_back(perception);
    }
  }

  if (perception_array.detections.size() > 0) {
    pub_->publish(perception_array);
  }
}

}  // namespace perception_system

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(perception_system::ObjectsDetectionNode)
