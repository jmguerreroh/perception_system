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

#include "perception_system/PersonPointingNode.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

namespace perception_system
{

PersonPointingNode::PersonPointingNode(const rclcpp::NodeOptions & options)
: rclcpp_cascade_lifecycle::CascadeLifecycleNode("person_pointing_node", options)
{
  // Add the activation of the people detection node
  this->add_activation("people_detection_node");
}

CallbackReturnT PersonPointingNode::on_configure(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(
    get_logger(), "[%s] Configuring from [%s] state...", get_name(),
    state.label().c_str());

  this->declare_parameter("debug", false);
  this->get_parameter("debug", debug_);
  this->declare_parameter("unique_id", -1);
  this->get_parameter("unique_id", unique_id_);

  pub_ = this->create_publisher<std_msgs::msg::UInt8>(
    "/perception_system/person_pointing", 10);

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT PersonPointingNode::on_activate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(
    get_logger(), "[%s] Activating from [%s] state...", get_name(),
    state.label().c_str());

  sub_ = this->create_subscription<yolov8_msgs::msg::DetectionArray>(
    "/perception_system/people_detection", 10,
    [this](yolov8_msgs::msg::DetectionArray::ConstSharedPtr msg) {return this->callback(msg);});

  pub_->on_activate();

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT PersonPointingNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(
    get_logger(), "[%s] Deactivating from [%s] state...", get_name(),
    state.label().c_str());

  sub_ = nullptr;

  pub_->on_deactivate();

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT PersonPointingNode::on_cleanup(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(
    get_logger(), "[%s] Cleaning up from [%s] state...", get_name(),
    state.label().c_str());

  pub_.reset();

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT PersonPointingNode::on_shutdown(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(
    get_logger(), "[%s] Shutting down from [%s] state...", get_name(),
    state.label().c_str());

  pub_.reset();

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT PersonPointingNode::on_error(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "[%s] Erroring from [%s] state...", get_name(), state.label().c_str());

  return CallbackReturnT::SUCCESS;
}

double distance2D(double x1, double y1, double x2, double y2)
{
  return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
}

int direction(double x1, double y1, double x2, double y2)
{
  // Calculate the change in x and y
  double deltaX = x2 - x1;
  double deltaY = y2 - y1;

  // Calculate the direction in radians
  double rad = std::atan2(deltaY, deltaX);

  // Change the direction to degrees
  double deg = fmod((rad * (180.0 / M_PI) + 360.0), 360.0);

  // Get the direction as a number between 0 and 7
  int num = static_cast<int>((deg + 22.5) / 45.0) % 8;

  return num;
}

void PersonPointingNode::callback(
  const yolov8_msgs::msg::DetectionArray::ConstSharedPtr & msg)
{

  if (msg->detections.size() == 0) {
    RCLCPP_INFO(get_logger(), "No detections");
    return;
  }

  cv::Mat img;
  if (debug_) {
    img = cv::Mat::zeros(480, 640, CV_8UC3);
  }

  auto global_detection = msg->detections[0];
  auto global_size = global_detection.bbox.size.x * global_detection.bbox.size.y;
  int64_t global_unique_id = getUniqueIDFromDetection(global_detection);
  float global_diff = diffIDs(unique_id_, global_unique_id);

  // Arms
  yolov8_msgs::msg::Point2D left_elbow, left_wrist, left_hip;
  yolov8_msgs::msg::Point2D right_elbow, right_wrist, right_hip;
  bool left_elbow_found = false;
  bool left_wrist_found = false;
  bool left_hip_found = false;
  bool right_elbow_found = false;
  bool right_wrist_found = false;
  bool right_hip_found = false;


  for (auto detection : msg->detections) {
    auto size = detection.bbox.size.x * detection.bbox.size.y;

    // Get the bounding box
    if (unique_id_ != -1) { // If the unique_id_ is set, we need to compare
      int64_t id = getUniqueIDFromDetection(detection);
      float min_diff = diffIDs(unique_id_, id);
      if (min_diff < global_diff) {
        // Display the results
        global_detection = detection;
        // global_size = size;
        global_unique_id = id;
        global_diff = min_diff;
      }
    } else { // If the unique_id_ is not set, we don't need to compare
      if (size >= global_size) {
        global_detection = detection;
        global_size = size;
      }
    }

    for (auto keypoint : global_detection.keypoints.data) {
      switch (keypoint.id) {
        case 8:
          left_elbow = keypoint.point;
          left_elbow_found = true;
          break;
        case 10:
          left_wrist = keypoint.point;
          left_wrist_found = true;
          break;
        case 12:
          left_hip = keypoint.point;
          left_hip_found = true;
          break;
        case 9:
          right_elbow = keypoint.point;
          right_elbow_found = true;
          break;
        case 11:
          right_wrist = keypoint.point;
          right_wrist_found = true;
          break;
        case 13:
          right_hip = keypoint.point;
          right_hip_found = true;
          break;

        default:
          break;
      }

      if (debug_) {
        cv::circle(
          img, cv::Point(keypoint.point.x, keypoint.point.y), 5, cv::Scalar(
            0, 255,
            0), -1);
        cv::putText(
          img, std::to_string(keypoint.id), cv::Point(
            keypoint.point.x,
            keypoint.point.y), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(
            255, 255, 255), 1);
      }
    }
  }

  bool left_arm = (left_elbow_found && left_wrist_found && left_hip_found);
  bool right_arm = (right_elbow_found && right_wrist_found && right_hip_found);

  if (!left_arm || !right_arm) {
    RCLCPP_INFO(get_logger(), "No arms found");
    return;
  }

  double left_distance = distance2D(left_hip.x, left_hip.y, left_wrist.x, left_wrist.y);
  double right_distance = distance2D(right_hip.x, right_hip.y, right_wrist.x, right_wrist.y);

  std_msgs::msg::UInt8 dir;

  if (left_distance < right_distance) {   // use right arm

    dir.data = direction(right_elbow.x, right_elbow.y, right_wrist.x, right_wrist.y);

    if (debug_) {
      RCLCPP_INFO(get_logger(), "Using right arm %f", right_distance);
      cv::circle(img, cv::Point(right_elbow.x, right_elbow.y), 5, cv::Scalar(0, 0, 255), -1);
      cv::circle(img, cv::Point(right_wrist.x, right_wrist.y), 5, cv::Scalar(0, 0, 255), -1);
      cv::line(
        img, cv::Point(right_elbow.x, right_elbow.y), cv::Point(
          right_wrist.x,
          right_wrist.y),
        cv::Scalar(0, 0, 255), 2);
    }
  } else {   // use left arm

    dir.data = direction(left_elbow.x, left_elbow.y, left_wrist.x, left_wrist.y);

    if (debug_) {
      RCLCPP_INFO(get_logger(), "Using left arm %f", left_distance);
      cv::circle(img, cv::Point(left_elbow.x, left_elbow.y), 5, cv::Scalar(255, 0, 0), -1);
      cv::circle(img, cv::Point(left_wrist.x, left_wrist.y), 5, cv::Scalar(255, 0, 0), -1);
      cv::line(
        img, cv::Point(left_elbow.x, left_elbow.y), cv::Point(
          left_wrist.x,
          left_wrist.y),
        cv::Scalar(255, 0, 0), 2);
    }
  }

  if (debug_) {
    cv::imshow("Person Pointing", img);

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
    cv::imshow("PT", roi);

    cv::waitKey(1);
  }

  pub_->publish(dir);
}

}  // namespace perception_system

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(perception_system::PersonPointingNode)
