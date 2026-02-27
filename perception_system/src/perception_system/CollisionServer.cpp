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
: rclcpp_cascade_lifecycle::CascadeLifecycleNode("collision_server", "perception_system", options)
{
  // todo: add dependencies

  // todo: declare params
  declare_parameter<std::string>("pointcloud_topic", "pointcloud_in");
  declare_parameter<std::string>("depth_topic", "depth_in");
  declare_parameter<std::string>("yolo_topic", "yolo_result");
  declare_parameter<std::string>("camera_info_topic", "depth_info");
  declare_parameter<float>("cluster_tolerance", 0.5);
  declare_parameter<float>("voxel_leaf_size", 0.01);
  declare_parameter<float>("erode_factor", 1.10);
  declare_parameter<int>("min_cluster_size", 50);
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
  get_parameter("erode_factor", erode_factor_);
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
  isolate_pc_background_service_ =
    create_service<perception_system_interfaces::srv::IsolatePCBackground>(
    "isolate_pc_background",
    std::bind(&CollisionServer::isolate_pc_background_service_callback, this, _1, _2));

  // Create subscribers

  remove_depth_classes_service_ =
    create_service<perception_system_interfaces::srv::RemoveDepthClasses>(
    "remove_classes_from_depth",
    std::bind(&CollisionServer::remove_classes_from_depth_callback, this, _1, _2));

  isolate_depth_classes_service_ =
    create_service<perception_system_interfaces::srv::IsolateDepthClasses>(
    "isolate_depth_classes",
    std::bind(&CollisionServer::isolate_classes_from_depth_callback, this, _1, _2));

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
    "pointcloud_filtered", 1);
  depth_pub_ = create_publisher<sensor_msgs::msg::Image>(
    "depth_filtered", 1);

  return CallbackReturn::SUCCESS;
}

CallbackReturn
CollisionServer::on_activate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(
    get_logger(), "[%s] Activating from [%s] state...", get_name(),
    state.label().c_str());

  point_cloud_pub_->on_activate();
  depth_pub_->on_activate();

  return CallbackReturn::SUCCESS;
}

CallbackReturn
CollisionServer::on_deactivate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(
    get_logger(), "[%s] Deactivating from [%s] state...", get_name(),
    state.label().c_str());

  point_cloud_pub_->on_deactivate();
  depth_pub_->on_deactivate();
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
  depth_pub_.reset();
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
  const yolo_msgs::msg::DetectionArray::ConstSharedPtr & yolo_result_msg)
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

bool CollisionServer::are_registered(
  yolo_msgs::msg::DetectionArray::ConstSharedPtr yolo_detection,
  sensor_msgs::msg::Image::ConstSharedPtr depth_image)
{
  return yolo_detection->header.frame_id == depth_image->header.frame_id;
}

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

  // erode holes:
  // std::vector< cv::Point2d> points;
  // for (auto const & point : plane.points) {
  //   cv::Point3d pt_cv(point.x, point.y, point.z);
  //   points.push_back(cam_model_.project3dToPixel(pt_cv));
  // }
  // cv::Mat_<cv::Point2d> projected_pc(points);
  // cv::Mat mask = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
  // cv::erode(projected_pc, projected_pc, mask);

  // pcl::PointCloud<pcl::PointXYZRGB>::Ptr eroded_plane(new pcl::PointCloud<pcl::PointXYZRGB>());

  // for (int i = 0; i < projected_pc.rows; i++) {
  //   for(int j = 0; j < projected_pc.cols; j++) {
  //     cv::Point2d point = projected_pc.at<cv::Point2d>(i, j);
  //     pcl::PointXYZRGB pcl_point;
  //     auto cv_point = cam_model_.projectPixelTo3dRay(point);
  //     pcl_point.x = cv_point.x;
  //     pcl_point.y = cv_point.y;
  //     pcl_point.z = cv_point.z;
  //     eroded_plane->push_back(pcl_point);
  //   }
  // }

  sensor_msgs::msg::PointCloud2 filtered_pc_;
  pcl::toROSMsg(plane, filtered_pc_);

  filtered_pc_.header = last_pc_->header;

  response->filtered_pc = filtered_pc_;
  response->success = true;
  point_cloud_pub_->publish(filtered_pc_);
}

std::shared_ptr<cv::Mat> CollisionServer::createMask(
  const std::vector<std::vector<cv::Point>> & contours,
  const cv::Size & size,
  const bool revert)
{
  // cv::Mat mask = cv::Mat::ones(size, CV_16SC1) * UINT16_MAX;
  cv::Mat mask = cv::Mat::ones(size, CV_8U) * UCHAR_MAX;
  if (revert) {
    mask.setTo(cv::Scalar(0));
  }

  cv::Scalar color = revert ? cv::Scalar(255) : cv::Scalar(0);

  cv::drawContours(mask, contours, -1, color, cv::FILLED);
  return std::make_shared<cv::Mat>(mask);
}

std::vector<std::vector<cv::Point>> CollisionServer::getCountours(
  yolo_msgs::msg::DetectionArray::ConstSharedPtr yolo_detection_msg)
{
  std::vector<std::vector<cv::Point>> contours;
  contours.reserve(yolo_detection_msg->detections.size());

  for (const auto & detection : yolo_detection_msg->detections) {
    RCLCPP_INFO(
      get_logger(),
      "Detected: %s", detection.class_name.c_str());
    std::vector<cv::Point> single_contour;
    single_contour.reserve(detection.mask.data.size());

    for (const auto & point : detection.mask.data) {
      single_contour.push_back(cv::Point(point.x, point.y));
    }
    contours.push_back(single_contour);
  }
  return contours;
}

bool CollisionServer::are_there_frames()
{
  // return last_pc_ != nullptr && last_yolo_ != nullptr && last_depth_image_ != nullptr;
  if (last_pc_ == nullptr || last_yolo_ == nullptr || last_depth_image_ == nullptr) {
    RCLCPP_ERROR(
      get_logger(),
      "No point cloud/yolo/depth received, cannot process collision extraction");
    if (last_pc_ == nullptr) {
      RCLCPP_ERROR(
        get_logger(),
        "No point cloud received, cannot process collision extraction");
    }
    if (last_yolo_ == nullptr) {
      RCLCPP_ERROR(
        get_logger(),
        "No yolo detection received, cannot process collision extraction");
    }
    if (last_depth_image_ == nullptr) {
      RCLCPP_ERROR(
        get_logger(),
        "No depth image received, cannot process collision extraction");
    }
    return false;
  }
  return true;
}

void CollisionServer::remove_classes_from_depth_callback(
  const std::shared_ptr<perception_system_interfaces::srv::RemoveDepthClasses::Request> request,
  std::shared_ptr<perception_system_interfaces::srv::RemoveDepthClasses::Response> response)
{
  if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    RCLCPP_WARN(
      get_logger(),
      "Received request but not in ACTIVE state, ignoring!");
  }
  if (!are_there_frames()) {
    response->success = false;
    return;
  }
  if (!are_registered(last_yolo_, last_depth_image_)) {
    response->success = false;
    RCLCPP_WARN(
      get_logger(),
      "The yolo detection and depth frame are not registered!");
    return;
  }

  cv_bridge::CvImagePtr image_depth_ptr;
  try {
    image_depth_ptr = cv_bridge::toCvCopy(last_depth_image_, "16UC1");
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(
      get_logger(),
      "Error during cv bridge image convertion: %s", e.what());
    response->success = false;
    return;
  }

  std::vector<std::vector<cv::Point>> contours = getCountours(last_yolo_);
  std::shared_ptr<cv::Mat> mask = createMask(contours, image_depth_ptr->image.size(), false);

  cv::Mat filtered_depth;
  image_depth_ptr->image.copyTo(filtered_depth, *mask);
  image_depth_ptr->image = filtered_depth;

  sensor_msgs::msg::Image::SharedPtr filtered_depth_msg = image_depth_ptr->toImageMsg();
  depth_pub_->publish(*filtered_depth_msg);

  cv_bridge::CvImage mask_msg;
  mask_msg.header = image_depth_ptr->header; // Same timestamp and tf frame as input image
  mask_msg.encoding = "8UC1";
  mask_msg.image = *mask;

  response->filtered_depth = *filtered_depth_msg;
  response->success = true;
  return;
}

void CollisionServer::isolate_classes_from_depth_callback(
  const std::shared_ptr<perception_system_interfaces::srv::IsolateDepthClasses::Request> request,
  std::shared_ptr<perception_system_interfaces::srv::IsolateDepthClasses::Response> response)
{
  if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    RCLCPP_WARN(
      get_logger(),
      "Received request but not in ACTIVE state, ignoring!");
  }

  if (!are_there_frames()) {
    response->success = false;
    return;
  }
  if (!are_registered(last_yolo_, last_depth_image_)) {
    response->success = false;
    RCLCPP_WARN(
      get_logger(),
      "The yolo detection and depth frame are not registered!");
    return;
  }

  cv_bridge::CvImagePtr image_depth_ptr;
  try {
    image_depth_ptr = cv_bridge::toCvCopy(last_depth_image_, "16UC1");
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(
      get_logger(),
      "Error during cv bridge image convertion: %s", e.what());
    response->success = false;
    return;
  }

  std::vector<std::vector<cv::Point>> contours = getCountours(last_yolo_);
  std::shared_ptr<cv::Mat> mask = createMask(contours, image_depth_ptr->image.size(), true);

  cv::Mat filtered_depth;
  image_depth_ptr->image.copyTo(filtered_depth, *mask);
  image_depth_ptr->image = filtered_depth;

  sensor_msgs::msg::Image::SharedPtr filtered_depth_msg = image_depth_ptr->toImageMsg();
  depth_pub_->publish(*filtered_depth_msg);

  cv_bridge::CvImage mask_msg;
  mask_msg.header = image_depth_ptr->header; // Same timestamp and tf frame as input image
  mask_msg.encoding = "8UC1";
  mask_msg.image = *mask;

  response->filtered_depth = *filtered_depth_msg;
  response->success = true;
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
  // Ensure yolo is not modified while executing the service
  yolo_msgs::msg::DetectionArray::ConstSharedPtr yolo_copy = last_yolo_;

  yolo_msgs::msg::DetectionArray filtered_yolo;

  if (request->ignore_class && !request->classes.empty()) {
    for (const auto & detection : yolo_copy->detections) {
      if (std::find(
          request->classes.begin(), request->classes.end(),
          detection.class_name) == request->classes.end())
      {
        filtered_yolo.detections.push_back(detection);
      }
    }
  } else if (!request->ignore_class && !request->classes.empty()) {
    for (const auto & detection : yolo_copy->detections) {
      if (std::find(
          request->classes.begin(), request->classes.end(),
          detection.class_name) != request->classes.end())
      {
        filtered_yolo.detections.push_back(detection);
      }
    }
  } else {
    filtered_yolo = *yolo_copy;
  }
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

  downsampled_cloud = downsampleCloudMsg(last_pc_, voxel_leaf_size_);
  sensor_msgs::msg::PointCloud2 detection_cloud_msg;

  auto const_filtered_yolo =
    std::make_shared<const yolo_msgs::msg::DetectionArray>(filtered_yolo);

  detection_cloud_msg = projectCloud(
    downsampled_cloud,
    const_filtered_yolo,
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
  //point_cloud_pub_->publish(detection_cloud_msg);
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

  // Ensure yolo is not modified while executing the service
  yolo_msgs::msg::DetectionArray::ConstSharedPtr yolo_copy = last_yolo_;

  yolo_msgs::msg::DetectionArray filtered_yolo;

  if (request->ignore_class && !request->classes.empty()) {
    for (const auto & detection : yolo_copy->detections) {
      if (std::find(
          request->classes.begin(), request->classes.end(),
          detection.class_name) == request->classes.end())
      {
        filtered_yolo.detections.push_back(detection);
      }
    }
  } else if (!request->ignore_class && !request->classes.empty()) {
    for (const auto & detection : yolo_copy->detections) {
      if (std::find(
          request->classes.begin(), request->classes.end(),
          detection.class_name) != request->classes.end())
      {
        filtered_yolo.detections.push_back(detection);
      }
    }
  } else {
    filtered_yolo = *yolo_copy;
  }

  auto const_filtered_yolo =
    std::make_shared<const yolo_msgs::msg::DetectionArray>(filtered_yolo);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

  downsampled_cloud = downsampleCloudMsg(last_pc_, voxel_leaf_size_);
  sensor_msgs::msg::PointCloud2 detection_cloud_msg;
  // Erotion only works on bb not on masks so far
  yolo_msgs::msg::DetectionArray::ConstSharedPtr eroded_yolo;
  eroded_yolo =
    std::make_shared<const yolo_msgs::msg::DetectionArray>(
    erodeDetections(
      const_filtered_yolo,
      erode_factor_));

  detection_cloud_msg = projectCloud(
    downsampled_cloud,
    eroded_yolo,
    cam_model_,
    cluster_tolerance_,
    min_cluster_size_,
    max_cluster_size_,
    last_pc_->header,
    true);

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

  // Publish the difference
  sensor_msgs::msg::PointCloud2 cloud_diff_msg;
  pcl::toROSMsg(*cloud_diff, cloud_diff_msg);
  cloud_diff_msg.header = last_pc_->header;

  response->filtered_pc = cloud_diff_msg;
  response->success = true;

  point_cloud_pub_->publish(cloud_diff_msg);
}


}  // namespace perception_system

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(perception_system::CollisionServer)
