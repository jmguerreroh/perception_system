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
#include "memory"

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "perception_system_interfaces/srv/extract_n_planes.hpp"
#include "perception_system_interfaces/srv/isolate_pc_classes.hpp"
#include "perception_system_interfaces/srv/isolate_pc_background.hpp"
#include "perception_system_interfaces/srv/remove_depth_classes.hpp"
#include "perception_system_interfaces/srv/isolate_depth_classes.hpp"

#include "yolo_msgs/msg/detection_array.hpp"

#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp_cascade_lifecycle/rclcpp_cascade_lifecycle.hpp"
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_geometry/pinhole_camera_model.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>

#include "perception_system/PerceptionUtils.hpp"

namespace perception_system
{

using CallbackReturn =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

using ApproximateSyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2,
    sensor_msgs::msg::Image,
    yolo_msgs::msg::DetectionArray>;

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
  // Callbacks
  void depth_info_cb(sensor_msgs::msg::CameraInfo info_msg);
  void sync_cb(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & pc_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg,
    const yolo_msgs::msg::DetectionArray::ConstSharedPtr & yolo_result_msg);
  void extract_n_planes_callback(
    const std::shared_ptr<perception_system_interfaces::srv::ExtractNPlanes::Request> request,
    std::shared_ptr<perception_system_interfaces::srv::ExtractNPlanes::Response> response);
  void isolate_pc_classes_service_callback(
    const std::shared_ptr<perception_system_interfaces::srv::IsolatePCClasses::Request> request,
    std::shared_ptr<perception_system_interfaces::srv::IsolatePCClasses::Response> response);
  void isolate_pc_background_service_callback(
    const std::shared_ptr<perception_system_interfaces::srv::IsolatePCBackground::Request> request,
    std::shared_ptr<perception_system_interfaces::srv::IsolatePCBackground::Response> response);
  void remove_classes_from_depth_callback(
    const std::shared_ptr<perception_system_interfaces::srv::RemoveDepthClasses::Request> request,
    std::shared_ptr<perception_system_interfaces::srv::RemoveDepthClasses::Response> response);
  void isolate_classes_from_depth_callback(
    const std::shared_ptr<perception_system_interfaces::srv::IsolateDepthClasses::Request> request,
    std::shared_ptr<perception_system_interfaces::srv::IsolateDepthClasses::Response> response);

  // Ros related members
  rclcpp::Service<perception_system_interfaces::srv::ExtractNPlanes>::SharedPtr
    extract_n_planes_service_;
  rclcpp::Service<perception_system_interfaces::srv::IsolatePCClasses>::SharedPtr
    isolate_pc_classes_service_;
  rclcpp::Service<perception_system_interfaces::srv::IsolatePCBackground>::SharedPtr
    isolate_pc_background_service_;
  rclcpp::Service<perception_system_interfaces::srv::RemoveDepthClasses>::SharedPtr
    remove_depth_classes_service_;
  rclcpp::Service<perception_system_interfaces::srv::IsolateDepthClasses>::SharedPtr
    isolate_depth_classes_service_;


  bool are_there_frames();
  std::shared_ptr<cv::Mat> createMask(
    const std::vector<std::vector<cv::Point>> & contours,
    const cv::Size & size,
    const bool revert);
  std::vector<std::vector<cv::Point>> getCountours(
    yolo_msgs::msg::DetectionArray::ConstSharedPtr yolo_detection_msg);
  bool are_registered(
    yolo_msgs::msg::DetectionArray::ConstSharedPtr yolo_detection,
    sensor_msgs::msg::Image::ConstSharedPtr depth_image);


  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;

  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2,
    rclcpp_lifecycle::LifecycleNode> pc_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image, rclcpp_lifecycle::LifecycleNode> depth_sub_;
  message_filters::Subscriber<yolo_msgs::msg::DetectionArray,
    rclcpp_lifecycle::LifecycleNode> yolo_result_sub_;
  std::shared_ptr<message_filters::Synchronizer<ApproximateSyncPolicy>> sync_;

  sensor_msgs::msg::PointCloud2::ConstSharedPtr last_pc_;
  yolo_msgs::msg::DetectionArray::ConstSharedPtr last_yolo_;
  sensor_msgs::msg::Image::ConstSharedPtr last_depth_image_;

  image_geometry::PinholeCameraModel cam_model_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Parameters
  std::string point_cloud_topic_;
  std::string depth_image_topic_;
  std::string yolo_result_topic_;
  std::string camera_info_topic_;

  float cluster_tolerance_;
  float voxel_leaf_size_;
  float erode_factor_;
  int min_cluster_size_;
  int max_cluster_size_;
};

}  // namespace perception_system

#endif  // COLLISION_SERVER_HPP_
