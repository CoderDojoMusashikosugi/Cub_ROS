// Copyright 2025 Cub Developer
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#ifndef CUB_MAPPING__MAP_CONVERTER_NODE_HPP_
#define CUB_MAPPING__MAP_CONVERTER_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/float64.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "cub_mapping/srv/add_region.hpp"

namespace cub_mapping
{

/**
 * @brief Backend node for converting 3D point clouds to 2D occupancy grid maps
 *
 * This node receives region selection requests from the RViz tool plugin,
 * processes the 3D point cloud data, and generates a 2D occupancy grid map
 * suitable for navigation with nav2.
 */
class MapConverterNode : public rclcpp::Node
{
public:
  MapConverterNode();
  virtual ~MapConverterNode() = default;

private:
  // Service callbacks
  void addRegionCallback(
    const std::shared_ptr<cub_mapping::srv::AddRegion::Request> request,
    std::shared_ptr<cub_mapping::srv::AddRegion::Response> response);

  void saveMapCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void clearMapCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  // Topic callbacks
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void sliceHeightCallback(const std_msgs::msg::Float64::SharedPtr msg);

  // Processing functions
  int processRegion(
    double center_x, double center_y, double center_z,
    double width, double height_range);

  void initializeOccupancyGrid();
  void updateOccupancyGrid(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_region,
    double center_x, double center_y, double width);

  void publishMap();
  void publishMapSliceTransform();
  void publishRegionMarker(
    double center_x, double center_y, double center_z,
    double width, double height_range, int region_id);

  bool saveMapToFile(const std::string & filename);

  // ROS2 communication
  rclcpp::Service<cub_mapping::srv::AddRegion>::SharedPtr add_region_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_map_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_map_service_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr region_marker_pub_;

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr slice_height_sub_;

  rclcpp::TimerBase::SharedPtr map_publish_timer_;
  rclcpp::TimerBase::SharedPtr tf_publish_timer_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Data storage
  pcl::PointCloud<pcl::PointXYZ>::Ptr full_pointcloud_;
  nav_msgs::msg::OccupancyGrid occupancy_grid_;

  // Parameters
  double resolution_;          // Map resolution (m/cell)
  double map_width_;           // Map width (m)
  double map_height_;          // Map height (m)
  double origin_x_;            // Map origin X (m)
  double origin_y_;            // Map origin Y (m)
  std::string map_frame_;      // Frame ID for the map
  std::string pointcloud_topic_;  // Point cloud topic to subscribe
  std::string output_directory_;  // Directory to save maps
  double publish_rate_;        // Map publish rate (Hz)
  double slice_height_;        // Current height of map_slice frame (m)
  double tf_publish_rate_;     // TF broadcast rate (Hz)

  // State
  bool map_initialized_;
  int region_counter_;
  rclcpp::Time last_pointcloud_time_;
};

}  // namespace cub_mapping

#endif  // CUB_MAPPING__MAP_CONVERTER_NODE_HPP_
