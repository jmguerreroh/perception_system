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

#pragma once

#include <iostream>
#include <algorithm>
#include <variant>
#include <vector>

#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_types_conversion.h"
#include "pcl/conversions.h"
#include <pcl/common/centroid.h>
#include "pcl/filters/voxel_grid.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/sample_consensus/ransac.h"
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <tf2_eigen/tf2_eigen.hpp>

#include <opencv2/opencv.hpp>
#include "yolo_msgs/msg/detection_array.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <image_geometry/pinhole_camera_model.h>
#include <cv_bridge/cv_bridge.h>

namespace perception_system
{

inline double distance3D(double x1, double y1, double z1, double x2, double y2, double z2)
{
  return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2) + std::pow(z2 - z1, 2));
}

inline double calculateMedian(const cv::Mat& channel) {
    // Flatten the channel matrix into a single vector
    std::vector<uchar> vecFromMat;
    if (channel.isContinuous()) {
        vecFromMat.assign(channel.datastart, channel.dataend);
    } else {
        for (int i = 0; i < channel.rows; ++i) {
            vecFromMat.insert(vecFromMat.end(), channel.ptr<uchar>(i), channel.ptr<uchar>(i) + channel.cols);
        }
    }

    // Sort the vector to find the median
    nth_element(vecFromMat.begin(), vecFromMat.begin() + vecFromMat.size() / 2, vecFromMat.end());
    double median;
    if (vecFromMat.size() % 2 == 0) {
        median = (vecFromMat[vecFromMat.size() / 2 - 1] + vecFromMat[vecFromMat.size() / 2]) / 2.0;
    } else {
        median = vecFromMat[vecFromMat.size() / 2];
    }

    return median;
}

inline cv::Scalar findMostCommonHSVColor(const cv::Mat& hsv_image)
{
    // Split the image into H, S, V channels
    std::vector<cv::Mat> hsv_channels;
    cv::split(hsv_image, hsv_channels);

    cv::Mat h_channel = hsv_channels[0];

    // Calculate the median of the H channel
    double h_mode = calculateMedian(h_channel);

    cv::InputArray lowerColor = cv::Scalar(h_mode - 5, 0, 0);
    cv::InputArray upperColor = cv::Scalar(h_mode + 5, 255, 255);

    cv::Mat mask;
    cv::inRange(hsv_image, lowerColor, upperColor, mask);
    auto masked_img = hsv_image.clone();
    cv::bitwise_and(hsv_image, hsv_image, masked_img, mask);

    std::vector<cv::Mat> proccessed_channels;
    cv::split(masked_img, proccessed_channels);

    cv::Mat s_channel = proccessed_channels[1];
    cv::Mat v_channel = proccessed_channels[2];

    cv::Scalar s_avg = cv::mean(s_channel);
    double s_value = s_avg[0];
    cv::Scalar v_avg = cv::mean(v_channel);
    double v_value = v_avg[0];

    return cv::Scalar(h_mode, s_value, v_value);
}

inline std::vector<cv::Scalar> calculateMedianHalves(const cv::Mat & roi)
{
  cv::Mat hsv;
  cv::cvtColor(roi, hsv, cv::COLOR_BGR2HSV);
  cv::Scalar up_avg = findMostCommonHSVColor(hsv(cv::Rect(0, 0, hsv.cols, hsv.rows / 2)));
  cv::Scalar down_avg = findMostCommonHSVColor(hsv(cv::Rect(0, hsv.rows / 2, hsv.cols, hsv.rows / 2)));
  
  return {up_avg, down_avg};
}

inline std::vector<cv::Scalar> calculateAverageHalves(const cv::Mat & roi)
{
  cv::Mat hsv;
  cv::cvtColor(roi, hsv, cv::COLOR_BGR2HSV);
  cv::Scalar up_avg = cv::mean(hsv(cv::Rect(0, 0, hsv.cols, hsv.rows / 2)));
  cv::Scalar down_avg = cv::mean(hsv(cv::Rect(0, hsv.rows / 2, hsv.cols, hsv.rows / 2)));
  return {up_avg, down_avg};
}

inline std::vector<cv::Scalar> getHSVFromUniqueID(int64_t uniqueID)
{
  std::vector<cv::Scalar> hsvColors(2);

  // Extract the components of the unique ID
  int64_t parte1 = uniqueID / static_cast<int64_t>(std::pow(10, 10));
  int64_t parte2 =
    (uniqueID % static_cast<int64_t>(std::pow(10, 10))) / static_cast<int64_t>(std::pow(10, 8));
  int64_t parte3 =
    (uniqueID % static_cast<int64_t>(std::pow(10, 8))) / static_cast<int64_t>(std::pow(10, 6));
  int64_t parte4 =
    (uniqueID % static_cast<int64_t>(std::pow(10, 6))) / static_cast<int64_t>(std::pow(10, 4));
  int64_t parte5 =
    (uniqueID % static_cast<int64_t>(std::pow(10, 4))) / static_cast<int64_t>(std::pow(10, 2));
  int64_t parte6 = uniqueID % static_cast<int64_t>(std::pow(10, 2));

  // Convert the components to the HSV color space
  double h01_1 = static_cast<float>(parte1) / 100 * 180;
  double s01_1 = static_cast<float>(parte2) / 100 * 256;
  double v01_1 = static_cast<float>(parte3) / 100 * 256;
  double h01_2 = static_cast<float>(parte4) / 100 * 180;
  double s01_2 = static_cast<float>(parte5) / 100 * 256;
  double v01_2 = static_cast<float>(parte6) / 100 * 256;

  // Assign the HSV colors to the output vector
  hsvColors[0] = cv::Scalar(h01_1, s01_1, v01_1);
  hsvColors[1] = cv::Scalar(h01_2, s01_2, v01_2);

  return hsvColors;
}

inline int64_t generateUniqueIDFromHSVPair(const cv::Scalar & hsv1, const cv::Scalar & hsv2)
{
  // Convert the HSV values to integers
  int64_t h01_1 = static_cast<int>(hsv1[0] / 180.0 * 100);        // Two decimal digits
  int64_t s01_1 = static_cast<int>(hsv1[1] / 256.0 * 100);
  int64_t v01_1 = static_cast<int>(hsv1[2] / 256.0 * 100);
  int64_t h01_2 = static_cast<int>(hsv2[0] / 180.0 * 100);
  int64_t s01_2 = static_cast<int>(hsv2[1] / 256.0 * 100);
  int64_t v01_2 = static_cast<int>(hsv2[2] / 256.0 * 100);

  // Create a unique ID by combining the components of both colors
  int64_t resultado = h01_1 * static_cast<int64_t>(std::pow(10, 10)) +
    s01_1 * static_cast<int64_t>(std::pow(10, 8)) +
    v01_1 * static_cast<int64_t>(std::pow(10, 6)) +
    h01_2 * static_cast<int64_t>(std::pow(10, 4)) +
    s01_2 * static_cast<int64_t>(std::pow(10, 2)) +
    v01_2;

  return static_cast<int64_t>(resultado);
}

inline std::vector<int8_t> mean_color(const cv::Mat & hsv)
{
  cv::Scalar avg = cv::mean(hsv);
  return {static_cast<int8_t>(avg[0]), static_cast<int8_t>(avg[1]), static_cast<int8_t>(avg[2])};
}

inline cv::Point2d checkPoint(cv::Point2d point, cv::Size size)
{
  if (point.x < 0) {
    point.x = 0;
  }
  if (point.y < 0) {
    point.y = 0;
  }
  if (point.x > size.width) {
    point.x = size.width;
  }
  if (point.y > size.height) {
    point.y = size.height;
  }
  return point;
}

inline int64_t getUniqueIDFromDetection(
  const sensor_msgs::msg::Image & img,
  const yolo_msgs::msg::Detection & detection)
{
  // Convert sensor_msgs::Image to cv::Mat using cv_bridge
  cv_bridge::CvImagePtr image_rgb_ptr;
  try {
    image_rgb_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception & e) {
    std::cerr << "cv_bridge exception: " << e.what() << std::endl;
    return -1;
  }

  cv::Mat image = image_rgb_ptr->image;

  // Get the bounding box
  auto bbox = detection.bbox;
  cv::Point2d min_pt = cv::Point2d(
    round(bbox.center.position.x - bbox.size.x / 2.0),
    round(bbox.center.position.y - bbox.size.y / 2.0));
  cv::Point2d max_pt = cv::Point2d(
    round(bbox.center.position.x + bbox.size.x / 2.0),
    round(bbox.center.position.y + bbox.size.y / 2.0));

  min_pt = checkPoint(min_pt, image.size());
  max_pt = checkPoint(max_pt, image.size());

  // Get the region of interest
  cv::Mat roi = image(cv::Rect(min_pt, max_pt));

  // Calculate the average color of the upper and lower halves
  std::vector<cv::Scalar> avg = calculateMedianHalves(roi);

  // Generate the unique ID
  return generateUniqueIDFromHSVPair(avg[0], avg[1]);
}


inline float diffIDs(int64_t id1, ino64_t id2)
{
  // Get the HSV values from the unique IDs
  std::vector<cv::Scalar> scalar1 = getHSVFromUniqueID(id1);
  std::vector<cv::Scalar> scalar2 = getHSVFromUniqueID(id2);

  cv::Scalar id1_up = scalar1[0];
  cv::Scalar id1_down = scalar1[1];

  cv::Scalar id2_up = scalar2[0];
  cv::Scalar id2_down = scalar2[1];

  cv::Scalar diff_h_up = abs(id1_up[0] - id2_up[0]);
  cv::Scalar diff_s_up = abs(id1_up[1] - id2_up[1]);
  cv::Scalar diff_v_up = abs(id1_up[2] - id2_up[2]);

  cv::Scalar diff_h_down = abs(id1_down[0] - id2_down[0]);
  cv::Scalar diff_s_down = abs(id1_down[1] - id2_down[1]);
  cv::Scalar diff_v_down = abs(id1_down[2] - id2_down[2]);

  // Return the average difference between the two unique IDs in HSV space with a weight of 2 for H and 1 for S and V
  return (diff_h_up[0] * 2 + diff_s_up[0] + diff_v_up[0] + diff_h_down[0] * 2 + diff_s_down[0] +
         diff_v_down[0]) / 6;
}

inline std::string underscore(std::string str)
{
  std::replace(str.begin(), str.end(), ' ', '_');
  return str;
}

inline double arm_distance(double x1, double y1, double x2, double y2)
{
  return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
}

inline int points_direction(double x1, double y1, double x2, double y2)
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

  // 0 is right, 1 is down-right, 2 is down, 3 is down-left, 4 is left, 5 is up-left, 6 is up, 7 is up-right
  return num;
}

inline int body_pose(yolo_msgs::msg::KeyPoint3DArray skeleton)
{
  geometry_msgs::msg::Point left_shoulder, left_hip, left_knee;
  geometry_msgs::msg::Point right_shoulder, right_hip, right_knee;

  bool left_shoulder_found = false;
  bool left_hip_found = false;
  bool left_knee_found = false;
  bool right_shoulder_found = false;
  bool right_hip_found = false;
  bool right_knee_found = false;

  for (auto keypoint : skeleton.data) {
    switch (keypoint.id) {
      case 5:
        left_shoulder = keypoint.point;
        left_shoulder_found = true;
        break;
      case 11:
        left_hip = keypoint.point;
        left_hip_found = true;
        break;
      case 13:
        left_knee = keypoint.point;
        left_knee_found = true;
        break;
      case 6:
        right_shoulder = keypoint.point;
        right_shoulder_found = true;
        break;
      case 12:
        right_hip = keypoint.point;
        right_hip_found = true;
        break;
      case 14:
        right_knee = keypoint.point;
        right_knee_found = true;
        break;

      default:
        break;
    }
  }

  bool left_leg = (left_hip_found && left_knee_found);
  bool right_leg = (right_hip_found && right_knee_found);
  bool left_body = (left_shoulder_found && left_hip_found);
  bool right_body = (right_shoulder_found && right_hip_found);

  double leg_position = -1;
  double body_position = -1;

  if (right_leg) {
    auto distance = sqrt(pow(right_hip.x - right_knee.x, 2) + pow(right_hip.y - right_knee.y, 2) + pow(right_hip.z - right_knee.z, 2));
    leg_position = (right_hip.y - right_knee.y) / distance;
  } else if (left_leg) {
    auto distance = sqrt(pow(left_hip.x - left_knee.x, 2) + pow(left_hip.y - left_knee.y, 2) + pow(left_hip.z - left_knee.z, 2));
    leg_position = (left_hip.y - left_knee.y) / distance;
  } else {
    return -1;
  }
  
  if (right_body) {
    auto distance = sqrt(pow(right_shoulder.x - right_hip.x, 2) + pow(right_shoulder.y - right_hip.y, 2) + pow(right_shoulder.z - right_hip.z, 2));
    body_position = (right_shoulder.y - right_hip.y) / distance;
  } else if (left_body) {
    auto distance = sqrt(pow(left_shoulder.x - left_hip.x, 2) + pow(left_shoulder.y - left_hip.y, 2) + pow(left_shoulder.z - left_hip.z, 2));
    body_position = (left_shoulder.y - left_hip.y) / distance;
  } else {
    return -1;
  }

  leg_position = abs(leg_position);
  body_position = abs(body_position);

  // 0 is lying, 1 is sitting, 2 is standing
  if (leg_position < 0.20 && body_position < 0.4) {
    return 2;
  } else if (leg_position > 0.20 && body_position < 0.4) {
    return 1;
  } else {
    return 0;
  }

}

inline int pointing(yolo_msgs::msg::KeyPoint2DArray skeleton)
{
  yolo_msgs::msg::Point2D left_elbow, left_wrist, left_hip;
  yolo_msgs::msg::Point2D right_elbow, right_wrist, right_hip;
  bool left_elbow_found = false;
  bool left_wrist_found = false;
  bool left_hip_found = false;
  bool right_elbow_found = false;
  bool right_wrist_found = false;
  bool right_hip_found = false;

  for (auto keypoint : skeleton.data) {
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
  }

  bool left_arm = (left_elbow_found && left_wrist_found && left_hip_found);
  bool right_arm = (right_elbow_found && right_wrist_found && right_hip_found);

  if (!left_arm || !right_arm) {
    // std::cout << "No arms found" << std::endl;
    return -1;
  }

  double left_distance = arm_distance(left_hip.x, left_hip.y, left_wrist.x, left_wrist.y);
  double right_distance = arm_distance(right_hip.x, right_hip.y, right_wrist.x, right_wrist.y);

  if (left_distance < right_distance) {   // use right arm
    return points_direction(right_elbow.x, right_elbow.y, right_wrist.x, right_wrist.y);
  } else {   // use left arm
    return points_direction(left_elbow.x, left_elbow.y, left_wrist.x, left_wrist.y);
  }
}

inline pcl::PointCloud<pcl::PointXYZRGB> extractNPlanes(
  const pcl::PointCloud<pcl::PointXYZRGB> in_pointcloud, const int & number_of_planes)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::copyPointCloud(in_pointcloud, *cloud_filtered);

  // Create the output point cloud
  pcl::PointCloud<pcl::PointXYZRGB> out_pointcloud;

  // Set the number of planes
  for (int i = 0; i < number_of_planes; ++i) {
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(0.01);

    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_p(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZRGB>);

    seg.setInputCloud(cloud_filtered);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());

    // Segment the largest plane
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() == 0) {
      std::cerr << "Could not estimate a plane model for the given dataset." << std::endl;
      break;      // Break the loop if no plane is found
    }

    // Extract the inliers - plane
    extract.setInputCloud(cloud_filtered);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*cloud_p);

    // Add the extracted plane to the output point cloud
    out_pointcloud += *cloud_p;

    // Remove the inliers from the input cloud to find the next plane
    extract.setNegative(true);
    extract.filter(*cloud_filtered);
  }

  // Perform StatisticalOutlierRemoval on the final output point cloud if needed
  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> f;
  f.setInputCloud(out_pointcloud.makeShared());
  f.setMeanK(50);
  f.setStddevMulThresh(3.0);
  f.filter(out_pointcloud);

  return out_pointcloud;
}

inline pcl::PointCloud<pcl::PointXYZRGB>::Ptr
downsampleCloudMsg(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud_msg,
  const double & voxel_leaf_size_)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::fromROSMsg(*cloud_msg, *cloud);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::VoxelGrid<pcl::PointXYZRGB>::Ptr voxel_grid(new pcl::VoxelGrid<pcl::PointXYZRGB>());
  voxel_grid->setInputCloud(cloud);
  voxel_grid->setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
  voxel_grid->filter(*downsampled_cloud);

  return downsampled_cloud;
}

inline pcl::PointCloud<pcl::PointXYZRGB>::Ptr
euclideanClusterExtraction(
  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
  const double & cluster_tolerance_, const uint16_t & min_cluster_size_,
  const uint16_t & max_cluster_size_)
{
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  ec.setClusterTolerance(cluster_tolerance_);
  ec.setMinClusterSize(min_cluster_size_);
  ec.setMaxClusterSize(max_cluster_size_);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud);
  ec.extract(cluster_indices);

  float min_distance = std::numeric_limits<float>::max();
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr closest_cluster(new pcl::PointCloud<pcl::PointXYZRGB>());

  for (const auto & cluster : cluster_indices) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>());
    for (const auto & indice : cluster.indices) {
      cloud_cluster->push_back((*cloud)[indice]);
    }

    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud_cluster, centroid);
    float distance = centroid.norm();

    if (distance < min_distance) {
      min_distance = distance;
      *closest_cluster = *cloud_cluster;
    }
  }

  return closest_cluster;
}

inline pcl::PointCloud<pcl::PointXYZRGB>::Ptr
processPointsWithBbox(
  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
  const yolo_msgs::msg::Detection & detection2d_msg,
  const image_geometry::PinholeCameraModel & cam_model_)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr ret(new pcl::PointCloud<pcl::PointXYZRGB>());

  for (const auto & point : cloud->points) {
    cv::Point3d pt_cv(point.x, point.y, point.z);
    cv::Point2d uv = cam_model_.project3dToPixel(pt_cv);

    if (point.z > 0 && uv.x > 0 &&
      uv.x >= detection2d_msg.bbox.center.position.x - detection2d_msg.bbox.size.x / 2 &&
      uv.x <= detection2d_msg.bbox.center.position.x + detection2d_msg.bbox.size.x / 2 &&
      uv.y >= detection2d_msg.bbox.center.position.y - detection2d_msg.bbox.size.y / 2 &&
      uv.y <= detection2d_msg.bbox.center.position.y + detection2d_msg.bbox.size.y / 2)
    {
      ret->points.push_back(point);
    }
  }
  return ret;
}

inline sensor_msgs::msg::Image
maskToImageMsg(const yolo_msgs::msg::Mask & mask_msg)
{
  sensor_msgs::msg::Image image_msg;

  image_msg.height = mask_msg.height;
  image_msg.width = mask_msg.width;
  image_msg.encoding = sensor_msgs::image_encodings::MONO8;
  image_msg.is_bigendian = false;
  image_msg.step = mask_msg.width;


  image_msg.data.resize(image_msg.width * image_msg.height, 0);

  std::vector<cv::Point> contour;
  for (const auto & point : mask_msg.data) {
    contour.emplace_back(static_cast<int>(point.x), static_cast<int>(point.y));
  }
  cv::Mat mask = cv::Mat::zeros(image_msg.height, image_msg.width, CV_8UC1);
  cv::drawContours(
    mask, std::vector<std::vector<cv::Point>>{contour}, 0, cv::Scalar(
      255), cv::FILLED);
  std::memcpy(image_msg.data.data(), mask.data, image_msg.data.size());


  return image_msg;
}


inline pcl::PointCloud<pcl::PointXYZRGB>::Ptr
processPointsWithMask(
  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
  const yolo_msgs::msg::Mask & mask_image_msg,
  const image_geometry::PinholeCameraModel & cam_model_,
  bool is_inverse = false
)
{
  sensor_msgs::msg::Image image_msg = maskToImageMsg(mask_image_msg);
  cv_bridge::CvImagePtr cv_ptr;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr ret(new pcl::PointCloud<pcl::PointXYZRGB>());

  try {
    cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::MONO8);
  } catch (cv_bridge::Exception & e) {
    std::cerr << e.what() << std::endl;
    return ret;
  }

  if (is_inverse) {
    cv::bitwise_not(cv_ptr->image, cv_ptr->image);
  }

  for (const auto & point : cloud->points) {
    cv::Point3d pt_cv(point.x, point.y, point.z);
    cv::Point2d uv = cam_model_.project3dToPixel(pt_cv);

    if (point.z > 0 && uv.x >= 0 && uv.x < mask_image_msg.width && uv.y >= 0 &&
      uv.y < mask_image_msg.height)
    {
      if (cv_ptr->image.at<uchar>(cv::Point(uv.x, uv.y)) > 0) {
        ret->points.push_back(point);
      }
    }
  }
  return ret;
}

inline sensor_msgs::msg::PointCloud2
projectCloud(
  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
  const yolo_msgs::msg::DetectionArray::ConstSharedPtr & yolo_result_msg,
  const image_geometry::PinholeCameraModel & cam_model_,
  const double & cluster_tolerance_,
  const uint16_t & min_cluster_size_,
  const uint16_t & max_cluster_size_,
  const std_msgs::msg::Header & header,
  bool erosion = false)
{
  sensor_msgs::msg::PointCloud2 combine_detection_cloud_msg;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr combine_detection_cloud(
    new pcl::PointCloud<pcl::PointXYZRGB>());

  for (size_t i = 0; i < yolo_result_msg->detections.size(); i++) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr detection_cloud_raw(
      new pcl::PointCloud<pcl::PointXYZRGB>());

    if (yolo_result_msg->detections[i].mask.data.empty() || erosion) {
      detection_cloud_raw =
        processPointsWithBbox(cloud, yolo_result_msg->detections[i], cam_model_);
    } else {
      detection_cloud_raw = processPointsWithMask(
        cloud, yolo_result_msg->detections[i].mask,
        cam_model_);
    }

    if (!detection_cloud_raw->points.empty()) {
      if (!erosion) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr closest_detection_cloud(
          new pcl::PointCloud<pcl::PointXYZRGB>());
        closest_detection_cloud = euclideanClusterExtraction(
          detection_cloud_raw, cluster_tolerance_,
          min_cluster_size_, max_cluster_size_);
        *combine_detection_cloud += *closest_detection_cloud;
      }
      *combine_detection_cloud += *detection_cloud_raw;

    } else {
      std::cerr << "Detection for " << yolo_result_msg->detections[i].class_name <<
        " couldnt be processed" << std::endl;
    }
  }
  pcl::toROSMsg(*combine_detection_cloud, combine_detection_cloud_msg);
  combine_detection_cloud_msg.header = header;
  return combine_detection_cloud_msg;
}

inline yolo_msgs::msg::DetectionArray
erodeDetections(
  const yolo_msgs::msg::DetectionArray::ConstSharedPtr & yolo_result_msg,
  const double & erode_factor)
{
  yolo_msgs::msg::DetectionArray eroded_yolo_result_msg;
  eroded_yolo_result_msg.header = yolo_result_msg->header;
  if (erode_factor == 1.0 || erode_factor <= 0.0 || yolo_result_msg->detections.empty()) {
    return *yolo_result_msg;
  }

  for (size_t i = 0; i < yolo_result_msg->detections.size(); i++) {
    yolo_msgs::msg::Detection detection = yolo_result_msg->detections[i];
    detection.bbox.size.x *= erode_factor;
    detection.bbox.size.y *= erode_factor;
    eroded_yolo_result_msg.detections.push_back(detection);

    if (yolo_result_msg->detections[i].mask.data.empty()) {
      continue;
    }
  }
  return eroded_yolo_result_msg;
}

} // namespace perception_system
