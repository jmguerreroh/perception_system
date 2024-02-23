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

#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_types_conversion.h"
#include "pcl/conversions.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/sample_consensus/ransac.h"
#include <pcl/filters/statistical_outlier_removal.h>

#include <opencv2/opencv.hpp>
#include "yolov8_msgs/msg/detection_array.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace perception_system
{
inline double distance3D(double x1, double y1, double z1, double x2, double y2, double z2)
{
  return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2) + std::pow(z2 - z1, 2));
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
  double s01_1 = static_cast<float>(parte2) / 100 * 255;
  double v01_1 = static_cast<float>(parte3) / 100 * 255;
  double h01_2 = static_cast<float>(parte4) / 100 * 180;
  double s01_2 = static_cast<float>(parte5) / 100 * 255;
  double v01_2 = static_cast<float>(parte6) / 100 * 255;

  // Assign the HSV colors to the output vector
  hsvColors[0] = cv::Scalar(h01_1, s01_1, v01_1);
  hsvColors[1] = cv::Scalar(h01_2, s01_2, v01_2);

  return hsvColors;
}

inline int64_t generateUniqueIDFromHSVPair(const cv::Scalar & hsv1, const cv::Scalar & hsv2)
{
  // Convert the HSV values to integers
  int64_t h01_1 = static_cast<int>(hsv1[0] / 180.0 * 100);        // Two decimal digits
  int64_t s01_1 = static_cast<int>(hsv1[1] / 255.0 * 100);
  int64_t v01_1 = static_cast<int>(hsv1[2] / 255.0 * 100);
  int64_t h01_2 = static_cast<int>(hsv2[0] / 180.0 * 100);
  int64_t s01_2 = static_cast<int>(hsv2[1] / 255.0 * 100);
  int64_t v01_2 = static_cast<int>(hsv2[2] / 255.0 * 100);

  // Create a unique ID by combining the components of both colors
  int64_t resultado = h01_1 * static_cast<int64_t>(std::pow(10, 10)) +
    s01_1 * static_cast<int64_t>(std::pow(10, 8)) +
    v01_1 * static_cast<int64_t>(std::pow(10, 6)) +
    h01_2 * static_cast<int64_t>(std::pow(10, 4)) +
    s01_2 * static_cast<int64_t>(std::pow(10, 2)) +
    v01_2;

  return static_cast<int64_t>(resultado);
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

inline int64_t getUniqueIDFromDetection(const yolov8_msgs::msg::Detection & detection)
{
  // Convert sensor_msgs::Image to cv::Mat using cv_bridge
  cv_bridge::CvImagePtr image_rgb_ptr;
  try {
    image_rgb_ptr = cv_bridge::toCvCopy(detection.source_img, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception & e) {
    std::cout << "cv_bridge exception: " << e.what() << std::endl;
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
  std::vector<cv::Scalar> avg = calculateAverageHalves(roi);

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

inline pcl::PointCloud<pcl::PointXYZRGB> ExtractNPlanes(
    const pcl::PointCloud<pcl::PointXYZRGB> in_pointcloud, const int & number_of_planes)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::copyPointCloud(in_pointcloud, *cloud_filtered);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_ptr_inliers(new pcl::PointCloud<pcl::PointXYZRGB>);

  // Create the filtering object
  pcl::VoxelGrid<pcl::PointXYZRGB> sor;
  sor.setInputCloud(cloud_filtered);
  sor.setLeafSize(0.01, 0.01, 0.01);
  sor.filter(*cloud_filtered_ptr_inliers);

  // Extract indices
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  // Optional
  seg.setOptimizeCoefficients(true);
  // Mandatory
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(1000);
  seg.setDistanceThreshold(0.01);


  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_p(new pcl::PointCloud<pcl::PointXYZRGB>), cloud_f(
    new pcl::PointCloud<pcl::PointXYZRGB>);
  // PointCloud with final indices
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr all_points(new pcl::PointCloud<pcl::PointXYZRGB>);

  seg.setInputCloud(cloud_filtered_ptr_inliers);
  seg.segment(*inliers, *coefficients);

  if (inliers->indices.size() == 0) {
    std::cerr << "Could not estimate a plane model for the given dataset." << std::endl;
    // break;
  }

  // Extract the inliers - plane
  extract.setInputCloud(cloud_filtered_ptr_inliers);
  extract.setIndices(inliers);
  extract.setNegative(false);
  extract.filter(*cloud_p);
  // std::cerr << "PointCloud representing the plane component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

  // Create the filtering object - points without plane
  extract.setNegative(true);
  extract.filter(*cloud_f);
  cloud_filtered_ptr_inliers.swap(cloud_f);
  // } while (cloud_p->size() > plane_size_);

  //----------------------------------------------------
  // StatisticalOutlierRemoval Filter
  //----------------------------------------------------
  // Create the filtering object
  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> f;
  // pass the input point cloud to the processing module
  f.setInputCloud (cloud_p);
  // set some parameters 
  f.setMeanK (50); 
  f.setStddevMulThresh (3.0);
  // get the output point cloud
  f.filter (*cloud_p);
  // std::cerr << "PointCloud inliners: " << cloud_filtered_ptr_inliers->width * cloud_filtered_ptr_inliers->height << " data points." << std::endl;

  // Create output pointcloud
  pcl::PointCloud<pcl::PointXYZRGB> out_pointcloud = *cloud_p;

  return out_pointcloud;
}

} // namespace perception_system
