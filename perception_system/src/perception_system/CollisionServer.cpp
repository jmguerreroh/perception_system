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
  last_yolo_ = nullptr;
  last_depth_image_ = nullptr;

  tf_buffer_  = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  collison_service_ = create_service<perception_system_interfaces::srv::ExtractNPlanes>(
    "extract_collision", std::bind(&CollisionServer::collision_service_callback, this, _1, _2));

  isolate_pc_classes_service_ = create_service<perception_system_interfaces::srv::IsolatePCClasses>(
    "isolate_pc_classes", std::bind(&CollisionServer::isolate_pc_classes_service_callback, this, _1, _2));

  remove_depth_classes_service_ = create_service<perception_system_interfaces::srv::RemoveDepthClasses>(
    "remove_classes_from_depth", std::bind(&CollisionServer::remove_classes_from_depth_callback, this, _1, _2));
  
  isolate_depth_classes_service_ = create_service<perception_system_interfaces::srv::IsolateDepthClasses>(
    "isolate_depth_classes", std::bind(&CollisionServer::isolate_classes_from_depth_callback, this, _1, _2));

  camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
    "depth_info",1,
    std::bind(&CollisionServer::depth_info_cb, this, std::placeholders::_1));

  pc_sub_.subscribe(shared_from_this(), "pointcloud_in");
  depth_sub_.subscribe(shared_from_this(), "depth_in");
  yolo_result_sub_.subscribe(shared_from_this(), "yolo_result");

  sync_ = std::make_shared<message_filters::Synchronizer<ApproximateSyncPolicy>>(10);
  sync_->connectInput(pc_sub_, depth_sub_, yolo_result_sub_);
  sync_->registerCallback(std::bind(&CollisionServer::sync_cb, this, _1,_2, _3));
  // latched topic  
  point_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    "pointcloud_filtered", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  depth_pub_ = create_publisher<sensor_msgs::msg::Image>(
    "depth_filtered", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  mask_pub_   = create_publisher<sensor_msgs::msg::Image>(
    "mask", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
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
  mask_pub_->on_activate();

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
  mask_pub_->on_deactivate();
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
  depth_pub_.reset();
  mask_pub_.reset();
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
CollisionServer::sync_cb(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& pc_msg,
                         const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg,                
                         const yolov8_msgs::msg::DetectionArray::ConstSharedPtr& yolo_result_msg)
{
  RCLCPP_INFO(get_logger(), "[%s] Sync Callback (new message) state...", get_name());
  last_pc_ = pc_msg;
  last_depth_image_ = depth_msg;
  last_yolo_ = yolo_result_msg;
}

void CollisionServer::depth_info_cb(sensor_msgs::msg::CameraInfo info_msg)
{
  cam_model_.fromCameraInfo(info_msg);
  camera_info_sub_ = nullptr;
}


bool CollisionServer::are_registered(yolov8_msgs::msg::DetectionArray::ConstSharedPtr yolo_detection,
                                     sensor_msgs::msg::Image::ConstSharedPtr depth_image)
{
  return yolo_detection->header.frame_id == depth_image->header.frame_id;
}

void CollisionServer::collision_service_callback(
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
    if (!are_registered(last_yolo_, last_depth_image_)) {
    RCLCPP_WARN(
      get_logger(),
      "The yolo detection and depth frame are not registered!");
    return;
  }
    
  pcl::PointCloud<pcl::PointXYZRGB> pointcloud;
  pcl::fromROSMsg(*last_pc_, pointcloud);

  auto plane = extractNPlanes(pointcloud, request->n_planes);
  sensor_msgs::msg::PointCloud2 filtered_pc_;
  pcl::toROSMsg(plane, filtered_pc_);

  filtered_pc_.header = last_pc_->header;

  response->filtered_pc = filtered_pc_;  

  response->success = true;

  point_cloud_pub_->publish(filtered_pc_);
}

std::shared_ptr<cv::Mat> CollisionServer::createMask(const std::vector<std::vector<cv::Point>>& contours, 
                                    const cv::Size& size,
                                    const bool revert)
{
  // cv::Mat mask = cv::Mat::ones(size, CV_16SC1) * UINT16_MAX;
  cv::Mat mask = cv::Mat::ones(size, CV_8U) * UCHAR_MAX; 
  if(revert)
  {
    mask.setTo(cv::Scalar(0)); 
  }

  cv::Scalar color = revert ? cv::Scalar(255) : cv::Scalar(0);

  cv::drawContours(mask, contours, -1, color, cv::FILLED);
  return std::make_shared<cv::Mat>(mask);
}

std::vector<std::vector<cv::Point>> CollisionServer::getCountours(yolov8_msgs::msg::DetectionArray::ConstSharedPtr yolo_detection_msg)
{
  std::vector<std::vector<cv::Point>> contours;
  contours.reserve(yolo_detection_msg->detections.size()); 

  for (const auto &detection : yolo_detection_msg->detections) {
    RCLCPP_INFO(
      get_logger(),
      "Detected: %s", detection.class_name.c_str());
    std::vector<cv::Point> single_contour;
    single_contour.reserve(detection.mask.data.size()); 

    for (const auto &point : detection.mask.data) {
      single_contour.push_back(cv::Point(point.x, point.y));
    }
  contours.push_back(single_contour);
  }
  return contours;
}

bool CollisionServer::are_there_frames()
{
  // return last_pc_ != nullptr && last_yolo_ != nullptr && last_depth_image_ != nullptr;
  if (last_pc_ == nullptr || last_yolo_ == nullptr || last_depth_image_ == nullptr)
  {
    RCLCPP_ERROR(
      get_logger(),
      "No point cloud/yolo/depth received, cannot process collision extraction");    
    if(last_pc_ == nullptr)
    {
      RCLCPP_ERROR(
        get_logger(),
        "No point cloud received, cannot process collision extraction");
    }
    if(last_yolo_ == nullptr)
    {
      RCLCPP_ERROR(
        get_logger(),
        "No yolo detection received, cannot process collision extraction");
    }
    if(last_depth_image_ == nullptr)
    {
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
  if(!are_there_frames())
  {
    response->success = false;
    return;
  }
  // if(!are_registered(last_yolo_, last_depth_image_))
  // {
  //   response->success = false;
  //   RCLCPP_WARN(
  //     get_logger(),
  //     "The yolo detection and depth frame are not registered!");
  //   return;
  // } 
  
  cv_bridge::CvImagePtr image_depth_ptr;
  try {
    image_depth_ptr = cv_bridge::toCvCopy(last_depth_image_, "16UC1");
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(
      get_logger(),
      "Error during cv bridge image convertion: %s", e.what());
    response->success = false;
    return ;
  }

  std::vector<std::vector<cv::Point>> contours = getCountours(last_yolo_);
  std::shared_ptr<cv::Mat> mask = createMask(contours, image_depth_ptr->image.size(),false);

  cv::Mat filtered_depth;
  image_depth_ptr->image.copyTo(filtered_depth, *mask);
  image_depth_ptr->image = filtered_depth;

  sensor_msgs::msg::Image::SharedPtr filtered_depth_msg = image_depth_ptr->toImageMsg();
  depth_pub_->publish(*filtered_depth_msg);

  cv_bridge::CvImage mask_msg;
  mask_msg.header = image_depth_ptr->header; // Same timestamp and tf frame as input image
  mask_msg.encoding = "8UC1";
  mask_msg.image    = *mask;
  mask_pub_->publish(*mask_msg.toImageMsg());


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
    
  if(!are_there_frames())
  {
    response->success = false;
    return;
  }
  // if(!are_registered(last_yolo_, last_depth_image_))
  // {
  //   response->success = false;
  //   RCLCPP_WARN(
  //     get_logger(),
  //     "The yolo detection and depth frame are not registered!");
  //   return;
  // } 

  cv_bridge::CvImagePtr image_depth_ptr;
  try {
    image_depth_ptr = cv_bridge::toCvCopy(last_depth_image_, "16UC1");
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(
      get_logger(),
      "Error during cv bridge image convertion: %s", e.what());
    response->success = false;
    return ;
  }

  std::vector<std::vector<cv::Point>> contours = getCountours(last_yolo_);
  std::shared_ptr<cv::Mat> mask = createMask(contours, image_depth_ptr->image.size(),true);

  cv::Mat filtered_depth;
  image_depth_ptr->image.copyTo(filtered_depth, *mask);
  image_depth_ptr->image = filtered_depth;

  sensor_msgs::msg::Image::SharedPtr filtered_depth_msg = image_depth_ptr->toImageMsg();
  depth_pub_->publish(*filtered_depth_msg);

  cv_bridge::CvImage mask_msg;
  mask_msg.header = image_depth_ptr->header; // Same timestamp and tf frame as input image
  mask_msg.encoding = "8UC1"; 
  mask_msg.image    = *mask; 

  mask_pub_->publish(*mask_msg.toImageMsg());

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
    
  if (last_pc_ == nullptr || last_yolo_ == nullptr || last_depth_image_ == nullptr ) {  
    response->success = false;
    RCLCPP_ERROR(
      get_logger(),
      "No point cloud/yolo/depth received, cannot process collision extraction");    
    return;
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_pc(new pcl::PointCloud<pcl::PointXYZ>());
  cv_bridge::CvImagePtr image_depth_ptr;
  try {
    image_depth_ptr = cv_bridge::toCvCopy(last_depth_image_, "32SC1");
  } catch (cv_bridge::Exception & e) {
    std::cout << "cv_bridge exception: " << e.what() << std::endl;
    return ;
  }

  for (const auto &detection : last_yolo_->detections) {
    for (const auto &point : detection.mask.data) {
      
      cv::Point2d mask_point(point.x, point.y);
      auto projected_mask = cam_model_.projectPixelTo3dRay(mask_point);
      projected_mask = projected_mask / projected_mask.z;
      // dividir el 3d point por el z
      // usar el depth como la profundiad
      pcl::PointXYZ point_;
      point_.x = projected_mask.x;
      point_.y = projected_mask.y;
      point_.z = projected_mask.z;
      filtered_pc->push_back(point_);
    }
  }

  sensor_msgs::msg::PointCloud2 filtered_cloud_msg;  
  pcl::toROSMsg(*filtered_pc, filtered_cloud_msg);
  filtered_cloud_msg.header = last_pc_->header;
  point_cloud_pub_->publish(filtered_cloud_msg);

  // pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  // // todo add voxel leaf size as a param
  // downsampled_cloud = downsampleCloudMsg(last_pc_, 0.05);
  // pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());

  // if (cam_model_.tfFrame() != last_pc_->header.frame_id)
  // {
  //   RCLCPP_INFO(
  //     get_logger(),
  //     "Transforming pointcloud");  
  //   // add a header to the as param 
  //   transformed_cloud = cloud2TransformedCloud(downsampled_cloud, tf_buffer_, last_pc_->header.frame_id, cam_model_.tfFrame());
  // }
  // else
  // {
  //   transformed_cloud = downsampled_cloud;
  // }

  // // // vision_msgs::msg::Detection3DArray detection3d_array_msg;
  // // sensor_msgs::msg::PointCloud2 detection_cloud_msg;
  // std::cout << "Isolating point cloud classes" << std::endl;
  // auto detection_cloud_msg = projectCloud(transformed_cloud, last_yolo_, cam_model_, tf_buffer_, 0.5, 100, 25000, last_pc_->header);

  // response->filtered_pc = detection_cloud_msg;
  //  response->success = true;
  // point_cloud_pub_->publish(detection_cloud_msg);
}

}  // namespace perception_system
