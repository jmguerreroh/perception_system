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
#include <pcl/segmentation/segment_differences.h>


using namespace std::chrono_literals;
using namespace std::placeholders;

namespace perception_system
{

CollisionServer::CollisionServer(const rclcpp::NodeOptions & options)
: rclcpp_cascade_lifecycle::CascadeLifecycleNode("collision_server_node", options)
{
  // todo: add dependencies

  // todo: declare params
  declare_parameter<std::string>("pointcloud_topic", "pointcloud_in");
  declare_parameter<std::string>("depth_topic", "depth_in");
  declare_parameter<std::string>("yolo_topic", "yolo_result");
  declare_parameter<std::string>("camera_info_topic", "depth_info");
  declare_parameter<float>("cluster_tolerance", 0.5);
  declare_parameter<float>("voxel_leaf_size", 0.01);
  declare_parameter<int>("min_cluster_size", 100);
  declare_parameter<int>("max_cluster_size", 25000);
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

  // Get parameters:
  get_parameter("pointcloud_topic", point_cloud_topic_);
  get_parameter("depth_topic", depth_image_topic_);
  get_parameter("yolo_topic", yolo_result_topic_);
  get_parameter("camera_info_topic", camera_info_topic_);
  get_parameter("cluster_tolerance", cluster_tolerance_);
  get_parameter("voxel_leaf_size", voxel_leaf_size_);
  get_parameter("min_cluster_size", min_cluster_size_);
  get_parameter("max_cluster_size", max_cluster_size_);

  // Initialize msgs
  last_pc_ = nullptr;
  last_yolo_ = nullptr;
  last_depth_image_ = nullptr;

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ =
    std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Create services

  extract_n_planes_service_ = create_service<perception_system_interfaces::srv::ExtractNPlanes>(
    "extract_n_planes", std::bind(&CollisionServer::extract_n_planes_callback, this, _1, _2));
  isolate_pc_classes_service_ = create_service<perception_system_interfaces::srv::IsolatePCClasses>(
    "isolate_pc_classes",
    std::bind(&CollisionServer::isolate_pc_classes_service_callback, this, _1, _2));
  isolate_pc_background_service_ = create_service<perception_system_interfaces::srv::IsolatePCBackground>(
    "isolate_pc_background",
    std::bind(&CollisionServer::isolate_pc_background_service_callback, this, _1, _2));

  // Create subscribers

  camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
    camera_info_topic_, 1,
    std::bind(&CollisionServer::depth_info_cb, this, std::placeholders::_1));
  pc_sub_.subscribe(shared_from_this(), point_cloud_topic_);
  depth_sub_.subscribe(shared_from_this(), depth_image_topic_);
  yolo_result_sub_.subscribe(shared_from_this(), yolo_result_topic_);

  sync_ = std::make_shared<message_filters::Synchronizer<ApproximateSyncPolicy>>(20);
  sync_->connectInput(pc_sub_, depth_sub_, yolo_result_sub_);
  sync_->registerCallback(std::bind(&CollisionServer::sync_cb, this, _1, _2, _3));
  // Latched topic
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
  extract_n_planes_service_.reset();
  isolate_pc_classes_service_.reset();
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

void
CollisionServer::sync_cb(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & pc_msg,
  const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg,
  const yolov8_msgs::msg::DetectionArray::ConstSharedPtr & yolo_result_msg)
{
  last_pc_ = pc_msg;
  last_depth_image_ = depth_msg;
  last_yolo_ = yolo_result_msg;
}

void CollisionServer::depth_info_cb(sensor_msgs::msg::CameraInfo info_msg)
{
  cam_model_.fromCameraInfo(info_msg);
  camera_info_sub_ = nullptr;
}

// todo: change the name of the service
void CollisionServer::extract_n_planes_callback(
  const std::shared_ptr<perception_system_interfaces::srv::ExtractNPlanes::Request> request,
  std::shared_ptr<perception_system_interfaces::srv::ExtractNPlanes::Response> response)
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
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  downsampled_cloud = downsampleCloudMsg(last_pc_, voxel_leaf_size_);

  auto plane = extractNPlanes(*downsampled_cloud, request->n_planes);

  sensor_msgs::msg::PointCloud2 filtered_pc_;
  pcl::toROSMsg(plane, filtered_pc_);

  filtered_pc_.header = last_pc_->header;

  response->filtered_pc = filtered_pc_;
  response->success = true;
  point_cloud_pub_->publish(filtered_pc_);
}

void CollisionServer::isolate_pc_classes_service_callback(
  const std::shared_ptr<perception_system_interfaces::srv::IsolatePCClasses::Request> request,
  std::shared_ptr<perception_system_interfaces::srv::IsolatePCClasses::Response> response)
{
  if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    RCLCPP_WARN(
      get_logger(),
      "Received request but not in ACTIVE state, ignoring!");
  }

  if (last_pc_ == nullptr || last_yolo_ == nullptr || last_depth_image_ == nullptr) {
    response->success = false;
    RCLCPP_ERROR(
      get_logger(),
      "No point cloud/yolo/depth received, cannot process collision extraction");
    return;
  }
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

  downsampled_cloud = downsampleCloudMsg(last_pc_, voxel_leaf_size_);
  sensor_msgs::msg::PointCloud2 detection_cloud_msg;

  detection_cloud_msg = projectCloud(
    downsampled_cloud,
    last_yolo_,
    cam_model_,
    cluster_tolerance_,
    min_cluster_size_,
    max_cluster_size_,
    last_pc_->header);

  if (detection_cloud_msg.data.empty()) {
    response->success = false;
    RCLCPP_ERROR(
      get_logger(),
      "No detection cloud generated, cannot process collision extraction, try changing the parameters");
    return;
  }
  response->filtered_pc = detection_cloud_msg;
  response->success = true;
  point_cloud_pub_->publish(detection_cloud_msg);
}

void CollisionServer::isolate_pc_background_service_callback(
  const std::shared_ptr<perception_system_interfaces::srv::IsolatePCBackground::Request> request,
  std::shared_ptr<perception_system_interfaces::srv::IsolatePCBackground::Response> response)
{
  if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    RCLCPP_WARN(
      get_logger(),
      "Received request but not in ACTIVE state, ignoring!");
  }

  if (last_pc_ == nullptr || last_yolo_ == nullptr || last_depth_image_ == nullptr) {
    response->success = false;
    RCLCPP_ERROR(
      get_logger(),
      "No point cloud/yolo/depth received, cannot process collision extraction");
    return;
  }
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

  downsampled_cloud = downsampleCloudMsg(last_pc_, voxel_leaf_size_);
  sensor_msgs::msg::PointCloud2 detection_cloud_msg;

  detection_cloud_msg = projectCloud(
    downsampled_cloud,
    last_yolo_,
    cam_model_,
    cluster_tolerance_,
    min_cluster_size_,
    max_cluster_size_,
    last_pc_->header);

  if (detection_cloud_msg.data.empty()) {
    response->success = false;
    RCLCPP_ERROR(
      get_logger(),
      "No detection cloud generated, cannot process collision extraction, try changing the parameters");
    return;
  }
    // ... Populate your point clouds ...

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_diff(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr mask(new pcl::PointCloud<pcl::PointXYZRGB>());

    pcl::fromROSMsg(detection_cloud_msg, *mask);

    // Create the SegmentDifferences object
    pcl::SegmentDifferences<pcl::PointXYZRGB> segment_diff;
    segment_diff.setInputCloud(downsampled_cloud);
    segment_diff.setTargetCloud(mask);

    // Segment the differences
    segment_diff.segment(*cloud_diff);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr closest_detection_cloud(
        new pcl::PointCloud<pcl::PointXYZRGB>());

    closest_detection_cloud = euclideanClusterExtraction(
      cloud_diff, 0.02,
      min_cluster_size_, max_cluster_size_);

    // Publish the difference
    sensor_msgs::msg::PointCloud2 cloud_diff_msg;
    pcl::toROSMsg(*closest_detection_cloud, cloud_diff_msg);
    cloud_diff_msg.header = last_pc_->header;

    point_cloud_pub_->publish(cloud_diff_msg);
}


}  // namespace perception_system
