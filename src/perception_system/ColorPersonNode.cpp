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

#include "perception_system/ColorPersonNode.hpp"

namespace perception_system
{

ColorPersonNode::ColorPersonNode()
: rclcpp_lifecycle::LifecycleNode("color_person_node") {}

CallbackReturnT ColorPersonNode::on_configure(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(
    get_logger(), "[%s] Configuring from [%s] state...", get_name(),
    state.label().c_str());

  this->declare_parameter("debug", false);
  this->get_parameter("debug", debug_);

  pub_ = this->create_publisher<std_msgs::msg::Int64>(
    "/perception_system/color_person", 10);

  markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/perception_system/bb_objects", 10);

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT ColorPersonNode::on_activate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(
    get_logger(), "[%s] Activating from [%s] state...", get_name(),
    state.label().c_str());

  sub_ = this->create_subscription<yolov8_msgs::msg::DetectionArray>(
    "/perception_system/people_detection", 10,
    [this](yolov8_msgs::msg::DetectionArray::ConstSharedPtr msg) {return this->callback(msg);});

  pub_->on_activate();
  markers_pub_->on_activate();

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT ColorPersonNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(
    get_logger(), "[%s] Deactivating from [%s] state...", get_name(),
    state.label().c_str());

  sub_ = nullptr;
  pub_->on_deactivate();
  markers_pub_->on_activate();

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT ColorPersonNode::on_cleanup(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(
    get_logger(), "[%s] Cleaning up from [%s] state...", get_name(),
    state.label().c_str());

  pub_.reset();
  markers_pub_.reset();

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT ColorPersonNode::on_shutdown(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(
    get_logger(), "[%s] Shutting down from [%s] state...", get_name(),
    state.label().c_str());

  pub_.reset();
  markers_pub_.reset();

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT ColorPersonNode::on_error(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "[%s] Erroring from [%s] state...", get_name(), state.label().c_str());

  return CallbackReturnT::SUCCESS;
}

void ColorPersonNode::callback(
  const yolov8_msgs::msg::DetectionArray::ConstSharedPtr & msg)
{
  // Find the closest person
  auto global_detection = msg->detections[0];
  auto global_dist = distance3D(
    global_detection.bbox3d.center.position.x,
    global_detection.bbox3d.center.position.y,
    global_detection.bbox3d.center.position.z, 0, 0, 0);

  for (auto detection : msg->detections) {
    auto dist_min = distance3D(
      detection.bbox3d.center.position.x,
      detection.bbox3d.center.position.y,
      detection.bbox3d.center.position.z, 0, 0, 0);

    if (dist_min < global_dist) {
      // Display the results
      global_detection = detection;
      global_dist = dist_min;
    }
  }

  // Convierte de sensor_msgs::Image a cv::Mat utilizando cv_bridge
  cv_bridge::CvImagePtr image_rgb_ptr;
  try {
    image_rgb_ptr = cv_bridge::toCvCopy(
      global_detection.source_img,
      sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  // Realiza operaciones con la imagen en formato cv::Mat
  cv::Mat image = image_rgb_ptr->image;

  cv::Point2d min_pt = cv::Point2d(
    round(global_detection.bbox.center.position.x - global_detection.bbox.size.x / 2.0),
    round(global_detection.bbox.center.position.y - global_detection.bbox.size.y / 2.0));
  cv::Point2d max_pt = cv::Point2d(
    round(global_detection.bbox.center.position.x + global_detection.bbox.size.x / 2.0),
    round(global_detection.bbox.center.position.y + global_detection.bbox.size.y / 2.0));

  min_pt = checkPoint(min_pt, image.size());
  max_pt = checkPoint(max_pt, image.size());

  cv::Mat roi = image(cv::Rect(min_pt, max_pt));

  // Convert the ROI to the HSV color space
  cv::Mat hsvRoi;
  cv::cvtColor(roi, hsvRoi, cv::COLOR_BGR2HSV);

  // Calculate the average color values (HSV) for each half
  std::vector<cv::Scalar> avg = calculateAverageHalves(roi);

  // Generate a unique identifier from the average color values
  int64_t uniqueID = generateUniqueIDFromHSVPair(avg[0], avg[1]);

  // Publish the unique identifier
  std_msgs::msg::Int64 msg_out;
  msg_out.data = uniqueID;

  pub_->publish(msg_out);

  if (debug_) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = msg->header.stamp;
    marker.ns = "perception_system";
    marker.id = 0;
    marker.text = "person";
    marker.frame_locked = false;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = global_detection.bbox3d.center.position.x;
    marker.pose.position.y = global_detection.bbox3d.center.position.y;
    marker.pose.position.z = global_detection.bbox3d.center.position.z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = global_detection.bbox3d.size.x;
    marker.scale.y = global_detection.bbox3d.size.y;
    marker.scale.z = global_detection.bbox3d.size.z;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    visualization_msgs::msg::MarkerArray marker_array;
    marker_array.markers.push_back(marker);
    markers_pub_->publish(marker_array);

    // Convierte de sensor_msgs::Image a cv::Mat utilizando cv_bridge
    cv_bridge::CvImagePtr image_rgb_ptr;
    try {
      image_rgb_ptr = cv_bridge::toCvCopy(
        global_detection.source_img,
        sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception & e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    // Realiza operaciones con la imagen en formato cv::Mat
    cv::Mat image = image_rgb_ptr->image;

    cv::Point2d min_pt = cv::Point2d(
      round(global_detection.bbox.center.position.x - global_detection.bbox.size.x / 2.0),
      round(global_detection.bbox.center.position.y - global_detection.bbox.size.y / 2.0));
    cv::Point2d max_pt = cv::Point2d(
      round(global_detection.bbox.center.position.x + global_detection.bbox.size.x / 2.0),
      round(global_detection.bbox.center.position.y + global_detection.bbox.size.y / 2.0));

    min_pt = checkPoint(min_pt, image.size());
    max_pt = checkPoint(max_pt, image.size());

    cv::Mat roi = image(cv::Rect(min_pt, max_pt));
    cv::imshow("ID", roi);
    cv::waitKey(1);
  }
}

}  // namespace perception_system
