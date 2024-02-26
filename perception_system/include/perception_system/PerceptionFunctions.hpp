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
#include "yolov8_msgs/msg/detection_array.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <image_geometry/pinhole_camera_model.h>

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

inline pcl::PointCloud<pcl::PointXYZRGB> extractNPlanes(
    const pcl::PointCloud<pcl::PointXYZRGB> in_pointcloud, const int & number_of_planes)
{
 pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
 pcl::copyPointCloud(in_pointcloud, *cloud_filtered);

  // Create the filtering object
  pcl::VoxelGrid<pcl::PointXYZRGB> sor;
  sor.setInputCloud(cloud_filtered);
  sor.setLeafSize(0.01, 0.01, 0.01);

  // Filter the input point cloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_ptr_inliers(new pcl::PointCloud<pcl::PointXYZRGB>);
  sor.filter(*cloud_filtered_ptr_inliers);

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

      seg.setInputCloud(cloud_filtered_ptr_inliers);
      pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
      pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
      
      // Segment the largest plane
      seg.segment(*inliers, *coefficients);

      if (inliers->indices.size() == 0) {
          std::cerr << "Could not estimate a plane model for the given dataset." << std::endl;
          break;  // Break the loop if no plane is found
      }

      // Extract the inliers - plane
      extract.setInputCloud(cloud_filtered_ptr_inliers);
      extract.setIndices(inliers);
      extract.setNegative(false);
      extract.filter(*cloud_p);

      // Add the extracted plane to the output point cloud
      out_pointcloud += *cloud_p;

      // Remove the inliers from the input cloud to find the next plane
      extract.setNegative(true);
      extract.filter(*cloud_filtered_ptr_inliers);
  }

  // Perform StatisticalOutlierRemoval on the final output point cloud if needed
  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> f;
  f.setInputCloud(out_pointcloud.makeShared());
  f.setMeanK(50);
  f.setStddevMulThresh(3.0);
  f.filter(out_pointcloud);

  return out_pointcloud;
  }

inline pcl::PointCloud<pcl::PointXYZ>::Ptr
downsampleCloudMsg(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_msg, const double& voxel_leaf_size_)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*cloud_msg, *cloud);

  pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::VoxelGrid<pcl::PointXYZ>::Ptr voxel_grid(new pcl::VoxelGrid<pcl::PointXYZ>());
  voxel_grid->setInputCloud(cloud);
  voxel_grid->setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
  voxel_grid->filter(*downsampled_cloud);

  return downsampled_cloud;
}

inline pcl::PointCloud<pcl::PointXYZ>::Ptr
euclideanClusterExtraction(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const double& cluster_tolerance_, const uint16_t& min_cluster_size_, const uint16_t& max_cluster_size_) 
{
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(cluster_tolerance_);
  ec.setMinClusterSize(min_cluster_size_);
  ec.setMaxClusterSize(max_cluster_size_);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud);
  ec.extract(cluster_indices);

  float min_distance = std::numeric_limits<float>::max();
  pcl::PointCloud<pcl::PointXYZ>::Ptr closest_cluster(new pcl::PointCloud<pcl::PointXYZ>());

  for (const auto& cluster : cluster_indices)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>());
    for (const auto& indice : cluster.indices)
    {
      cloud_cluster->push_back((*cloud)[indice]);
    }

    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud_cluster, centroid);
    float distance = centroid.norm();

    if (distance < min_distance)
    {
      min_distance = distance;
      *closest_cluster = *cloud_cluster;
    }
  }

  return closest_cluster;
}

inline void
transformPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,
                    pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out,
                    const Eigen::Affine3f& transform)
{
  int cloud_size = cloud_in->size();
  cloud_out->resize(cloud_size);

  for (int i = 0; i < cloud_size; i++)
  {
    const auto& point = cloud_in->points[i];
    cloud_out->points[i].x =
        transform(0, 0) * point.x + transform(0, 1) * point.y + transform(0, 2) * point.z + transform(0, 3);
    cloud_out->points[i].y =
        transform(1, 0) * point.x + transform(1, 1) * point.y + transform(1, 2) * point.z + transform(1, 3);
    cloud_out->points[i].z =
        transform(2, 0) * point.x + transform(2, 1) * point.y + transform(2, 2) * point.z + transform(2, 3);
  }
}

inline pcl::PointCloud<pcl::PointXYZ>::Ptr
cloud2TransformedCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                       const std::shared_ptr<tf2_ros::Buffer>& tf_buffer_,
                       const std::string& source_frame, const std::string& target_frame
                      )
{
  try
  {
    geometry_msgs::msg::TransformStamped tf_stamped = tf_buffer_->lookupTransform(target_frame, source_frame, tf2::TimePointZero);
    Eigen::Affine3f eigen_transform = tf2::transformToEigen(tf_stamped.transform).cast<float>();
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    transformPointCloud(cloud, transformed_cloud, eigen_transform);

    return transformed_cloud;
  }
  catch (tf2::TransformException& e)
  {
    std::cerr << e.what() << std::endl;
    return cloud;
  }
}

inline void
processPointsWithBbox(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                      const yolov8_msgs::msg::Detection& detection2d_msg,
                      const image_geometry::PinholeCameraModel& cam_model_,
                      pcl::PointCloud<pcl::PointXYZ>::Ptr& detection_cloud_raw)
{
  for (const auto& point : cloud->points)
  {
    cv::Point3d pt_cv(point.x, point.y, point.z);
    cv::Point2d uv = cam_model_.project3dToPixel(pt_cv);

    if (point.z > 0 && uv.x > 0 && uv.x >= detection2d_msg.bbox.center.position.x - detection2d_msg.bbox.size.x / 2 &&
        uv.x <= detection2d_msg.bbox.center.position.x + detection2d_msg.bbox.size.x / 2 &&
        uv.y >= detection2d_msg.bbox.center.position.y - detection2d_msg.bbox.size.y / 2 &&
        uv.y <= detection2d_msg.bbox.center.position.y + detection2d_msg.bbox.size.y / 2)
    {
      detection_cloud_raw->points.push_back(point);
    }
  }
}

inline sensor_msgs::msg::Image
maskToImageMsg(const yolov8_msgs::msg::Mask& mask_msg)
{
  sensor_msgs::msg::Image image_msg;

  image_msg.height = mask_msg.height;
  image_msg.width = mask_msg.width;
  image_msg.encoding = sensor_msgs::image_encodings::MONO8;
  image_msg.is_bigendian = false;
  image_msg.step = mask_msg.width;
  

  image_msg.data.resize(image_msg.width * image_msg.height);
        for (const auto& point : mask_msg.data) {
            // Assuming point.x and point.y are integers representing pixel coordinates
            size_t pixel_index = static_cast<size_t>(point.y) * image_msg.width + static_cast<size_t>(point.x);
            if (pixel_index < image_msg.data.size()) {  
                image_msg.data[pixel_index] = 255;  
            }
  }


  return image_msg;
}


inline pcl::PointCloud<pcl::PointXYZ>::Ptr 
processPointsWithMask(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                      const yolov8_msgs::msg::Mask& mask_image_msg,
                      const image_geometry::PinholeCameraModel& cam_model_
                     )
{
  // sensor_msgs::msg::Image image_msg = maskToImageMsg(mask_image_msg);
  // cv_bridge::CvImagePtr cv_ptr;
  pcl::PointCloud<pcl::PointXYZ>::Ptr ret (new pcl::PointCloud<pcl::PointXYZ>());

  // try
  // {
  //   cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::MONO8);
  // }
  // catch (cv_bridge::Exception& e)
  // {
  //   std::cerr << e.what()<< std::endl;
  //   return ret;
  // }
  // std::cout << "Image converted from msg to cv" << std::endl;
  // cv::imshow("mask", cv_ptr->image);
  // cv::waitKey(0);

  for (const auto& point : cloud->points)
  {
    // std::cout << "Translading points from image to pc" << std::endl; it enteres to it
    cv::Point3d pt_cv(point.x, point.y, point.z);
    cv::Point2d uv = cam_model_.project3dToPixel(pt_cv);

    if (point.z > 0 && uv.x >= 0 && uv.x < mask_image_msg.width && uv.y >= 0 && uv.y < mask_image_msg.height)
    {
      // std::cout << "point inside the image" << std::endl;
      auto it = std::find_if(mask_image_msg.data.begin(), mask_image_msg.data.end(), [&uv](yolov8_msgs::msg::Point2D point) {
        // std::cout << "Comparing points" << std::endl;
        std::cout << "point.x: " << point.x << " uv.x: " << uv.x << " point.y: " << point.y << " uv.y: " << uv.y << std::endl;
        return point.x == uv.x && point.y == uv.y;      
      });
      if (it != mask_image_msg.data.end())
      {
        std::cout << "segmeting one point of the pc" << std::endl;
        ret->points.push_back(point);
      }
      

      // std::cout << "point is in image mask at" << std::endl;
      // if (cv_ptr->image.at<uchar>(cv::Point(uv.x, uv.y)) > 0)
      // {
      //   std::cout << "pushing points into the new pc" << std::endl;
      //   ret->points.push_back(point);
      // }
    }
  }
  return ret;
}

inline sensor_msgs::msg::PointCloud2 
projectCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
            const yolov8_msgs::msg::DetectionArray::ConstSharedPtr& yolo_result_msg,
            const image_geometry::PinholeCameraModel& cam_model_,
            const std::shared_ptr<tf2_ros::Buffer>& tf_buffer_,
            const double& cluster_tolerance_,
            const uint16_t& min_cluster_size_,
            const uint16_t& max_cluster_size_,
            const std_msgs::msg::Header& header
           )
{
  sensor_msgs::msg::PointCloud2 combine_detection_cloud_msg;
  pcl::PointCloud<pcl::PointXYZ>::Ptr combine_detection_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  // detection3d_array_msg.header = header;
  // detection3d_array_msg.header.stamp = yolo_result_msg->header.stamp;

  for (size_t i = 0; i < yolo_result_msg->detections.size(); i++)
  {
    std::cout << "Processing yolo detections" << std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr detection_cloud_raw(new pcl::PointCloud<pcl::PointXYZ>());

    if (yolo_result_msg->detections[i].mask.data.empty())
    {
      std::cout << "Processing detection with bbox" << std::endl;
      processPointsWithBbox(cloud, yolo_result_msg->detections[i], cam_model_,  detection_cloud_raw);
    }
    else
    {
      std::cout << "Processing detection with mask" << std::endl;
      detection_cloud_raw =processPointsWithMask(cloud, yolo_result_msg->detections[i].mask, cam_model_);
    }

    if (!detection_cloud_raw->points.empty())
    {
      // pcl::PointCloud<pcl::PointXYZ>::Ptr detection_cloud(new pcl::PointCloud<pcl::PointXYZ>());
      // detection_cloud =
      //     cloud2TransformedCloud(detection_cloud_raw, tf_buffer_, cam_model_.tfFrame(), header.frame_id);

      pcl::PointCloud<pcl::PointXYZ>::Ptr closest_detection_cloud(new pcl::PointCloud<pcl::PointXYZ>());
      closest_detection_cloud = euclideanClusterExtraction(detection_cloud_raw, cluster_tolerance_, min_cluster_size_, max_cluster_size_);
      *combine_detection_cloud += *closest_detection_cloud;

      // createBoundingBox(detection3d_array_msg, closest_detection_cloud,
      //                   yolo_result_msg->.detections[i].results);
    }
    else
    {
      std::cerr << "Detection " << i << " couldnt be processed" << std::endl;
    }
    
  }

  pcl::toROSMsg(*combine_detection_cloud, combine_detection_cloud_msg);
  combine_detection_cloud_msg.header = header;
  return combine_detection_cloud_msg;
}

} // namespace perception_system
