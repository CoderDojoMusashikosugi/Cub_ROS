// Copyright 2025 Cub Developer
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#ifndef CUB_MAPPING__TERRAIN_ANALYSIS_HPP_
#define CUB_MAPPING__TERRAIN_ANALYSIS_HPP_

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

namespace cub_mapping
{

/**
 * @brief Terrain analysis node for 2D obstacle detection from PCD files
 *
 * This node processes a full 3D point cloud (loaded from PCD) and generates
 * a 2D OccupancyGrid for navigation. It analyzes terrain height and marks
 * obstacles based on elevation differences from ground.
 */
class TerrainAnalysis : public rclcpp::Node
{
public:
  TerrainAnalysis();
  virtual ~TerrainAnalysis() = default;

private:
  // Subscription callbacks
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud);

  // Service callbacks
  void generateMapCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void saveMapCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  // Processing functions
  void processFullPointCloud();
  void initializeGrids();
  void estimateGroundElevation();
  void generateOccupancyGrid();
  void publishResults();
  bool saveMapToFile(const std::string & filename);

  // ROS2 communication
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_pub_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr generate_map_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_map_service_;

  rclcpp::TimerBase::SharedPtr publish_timer_;

  // Parameters
  double resolution_;           // OccupancyGrid resolution (m/cell)
  double obstacle_height_min_;  // Minimum height above ground to be obstacle (m)
  double obstacle_height_max_;  // Maximum height above ground to be obstacle (m)
  double ground_clearance_;     // Height tolerance for ground detection (m)
  double quantile_z_;           // Quantile for ground estimation (0.0-1.0)
  int min_points_per_cell_;     // Minimum points to determine cell status
  std::string map_frame_;
  std::string pointcloud_topic_;
  std::string output_directory_;
  bool auto_generate_;          // Auto-generate map when PCD received

  // Grid parameters (computed from point cloud bounds)
  double origin_x_, origin_y_;
  double map_width_, map_height_;
  int grid_width_, grid_height_;

  // Point cloud data
  pcl::PointCloud<pcl::PointXYZ>::Ptr full_pointcloud_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloud_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_cloud_;

  // Grid data
  std::vector<std::vector<float>> cell_heights_;  // All heights per cell
  std::vector<float> ground_elevation_;           // Estimated ground per cell
  nav_msgs::msg::OccupancyGrid occupancy_grid_;   // Final occupancy grid

  // State
  bool pointcloud_received_;
  bool map_generated_;

  // Utility functions
  void loadParameters();
  int getGridIndex(int grid_x, int grid_y) const;
  bool isInBounds(int grid_x, int grid_y) const;
  void getGridCoords(double x, double y, int & grid_x, int & grid_y) const;
};

}  // namespace cub_mapping

#endif  // CUB_MAPPING__TERRAIN_ANALYSIS_HPP_#endif  // CUB_MAPPING__TERRAIN_ANALYSIS_HPP_
