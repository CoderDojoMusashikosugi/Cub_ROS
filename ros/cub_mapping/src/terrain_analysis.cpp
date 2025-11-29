// Copyright 2025 Cub Developer
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "cub_mapping/terrain_analysis.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <sys/stat.h>

#include <opencv2/opencv.hpp>
#include <pcl/common/common.h>

namespace cub_mapping
{

TerrainAnalysis::TerrainAnalysis()
: rclcpp::Node("terrain_analysis"),
  full_pointcloud_(new pcl::PointCloud<pcl::PointXYZ>()),
  ground_cloud_(new pcl::PointCloud<pcl::PointXYZ>()),
  obstacle_cloud_(new pcl::PointCloud<pcl::PointXYZ>()),
  pointcloud_received_(false),
  map_generated_(false)
{
  // Load parameters
  loadParameters();

  // Create subscription
  cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    pointcloud_topic_, 10,
    std::bind(&TerrainAnalysis::pointCloudCallback, this, std::placeholders::_1));

  // Create publishers
  occupancy_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "/terrain_occupancy_grid", 10);

  ground_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/ground_cloud", 10);

  obstacle_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/obstacle_cloud", 10);

  // Create services
  generate_map_service_ = this->create_service<std_srvs::srv::Trigger>(
    "generate_terrain_map",
    std::bind(&TerrainAnalysis::generateMapCallback, this,
              std::placeholders::_1, std::placeholders::_2));

  save_map_service_ = this->create_service<std_srvs::srv::Trigger>(
    "save_terrain_map",
    std::bind(&TerrainAnalysis::saveMapCallback, this,
              std::placeholders::_1, std::placeholders::_2));

  // Create publish timer
  auto timer_period = std::chrono::duration<double>(1.0);  // 1 Hz
  publish_timer_ = this->create_wall_timer(
    timer_period,
    std::bind(&TerrainAnalysis::publishResults, this));

  RCLCPP_INFO(this->get_logger(), "TerrainAnalysis node initialized");
  RCLCPP_INFO(this->get_logger(), "  Subscribing to: %s", pointcloud_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Resolution: %.3f m/cell", resolution_);
  RCLCPP_INFO(this->get_logger(), "  Obstacle height: %.2f - %.2f m", obstacle_height_min_, obstacle_height_max_);
  RCLCPP_INFO(this->get_logger(), "  Auto-generate: %s", auto_generate_ ? "true" : "false");
}

void TerrainAnalysis::loadParameters()
{
  resolution_ = this->declare_parameter<double>("resolution", 0.2);
  obstacle_height_min_ = this->declare_parameter<double>("obstacle_height_min", 0.15);
  obstacle_height_max_ = this->declare_parameter<double>("obstacle_height_max", 2.0);
  ground_clearance_ = this->declare_parameter<double>("ground_clearance", 0.1);
  quantile_z_ = this->declare_parameter<double>("quantile_z", 0.25);
  min_points_per_cell_ = this->declare_parameter<int>("min_points_per_cell", 3);
  map_frame_ = this->declare_parameter<std::string>("map_frame", "map");
  pointcloud_topic_ = this->declare_parameter<std::string>("pointcloud_topic", "/cloud_pcd");
  output_directory_ = this->declare_parameter<std::string>("output_directory", "/tmp/terrain_maps");
  auto_generate_ = this->declare_parameter<bool>("auto_generate", true);
}

void TerrainAnalysis::pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud)
{
  // Convert ROS message to PCL
  full_pointcloud_->clear();
  pcl::fromROSMsg(*cloud, *full_pointcloud_);

  pointcloud_received_ = true;
  map_generated_ = false;

  RCLCPP_INFO(this->get_logger(), "Received point cloud with %zu points", full_pointcloud_->size());

  // Auto-generate map if enabled
  if (auto_generate_ && !full_pointcloud_->empty()) {
    processFullPointCloud();
  }
}

void TerrainAnalysis::generateMapCallback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  if (!pointcloud_received_ || full_pointcloud_->empty()) {
    response->success = false;
    response->message = "No point cloud data available";
    return;
  }

  processFullPointCloud();

  response->success = true;
  response->message = "Terrain map generated successfully";
}

void TerrainAnalysis::saveMapCallback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  if (!map_generated_) {
    response->success = false;
    response->message = "No map generated yet";
    return;
  }

  // Create timestamped directory
  auto now = this->now();
  std::time_t time_t_seconds = static_cast<std::time_t>(now.seconds());
  std::tm * tm_local = std::localtime(&time_t_seconds);

  std::stringstream dir_ss;
  dir_ss << output_directory_ << "/terrain_"
         << std::setfill('0')
         << std::setw(4) << (tm_local->tm_year + 1900)
         << std::setw(2) << (tm_local->tm_mon + 1)
         << std::setw(2) << tm_local->tm_mday
         << "_"
         << std::setw(2) << tm_local->tm_hour
         << std::setw(2) << tm_local->tm_min
         << std::setw(2) << tm_local->tm_sec;
  std::string map_directory = dir_ss.str();

  // Create directory recursively using system command
  std::string mkdir_cmd = "mkdir -p \"" + map_directory + "\"";
  int ret = std::system(mkdir_cmd.c_str());
  if (ret != 0) {
    response->success = false;
    response->message = "Failed to create directory: " + map_directory;
    RCLCPP_ERROR(this->get_logger(), "mkdir -p failed for: %s", map_directory.c_str());
    return;
  }

  std::string base_filename = map_directory + "/map";
  bool success = saveMapToFile(base_filename);

  if (success) {
    response->success = true;
    response->message = "Map saved to " + base_filename + ".pgm/.yaml";
  } else {
    response->success = false;
    response->message = "Failed to save map";
  }
}

void TerrainAnalysis::processFullPointCloud()
{
  if (full_pointcloud_->empty()) {
    RCLCPP_WARN(this->get_logger(), "Point cloud is empty");
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Processing full point cloud...");

  // Initialize grids based on point cloud bounds
  initializeGrids();

  // Estimate ground elevation for each cell
  estimateGroundElevation();

  // Generate occupancy grid
  generateOccupancyGrid();

  map_generated_ = true;

  RCLCPP_INFO(this->get_logger(), "Terrain map generated: %d x %d cells", grid_width_, grid_height_);
  RCLCPP_INFO(this->get_logger(), "  Ground points: %zu", ground_cloud_->size());
  RCLCPP_INFO(this->get_logger(), "  Obstacle points: %zu", obstacle_cloud_->size());
}

void TerrainAnalysis::initializeGrids()
{
  // Compute bounding box with margin
  pcl::PointXYZ min_pt, max_pt;
  pcl::getMinMax3D(*full_pointcloud_, min_pt, max_pt);

  double margin = 1.0;  // 1m margin
  origin_x_ = min_pt.x - margin;
  origin_y_ = min_pt.y - margin;
  map_width_ = (max_pt.x - min_pt.x) + 2 * margin;
  map_height_ = (max_pt.y - min_pt.y) + 2 * margin;

  grid_width_ = static_cast<int>(std::ceil(map_width_ / resolution_));
  grid_height_ = static_cast<int>(std::ceil(map_height_ / resolution_));

  // Initialize data structures
  int num_cells = grid_width_ * grid_height_;
  cell_heights_.clear();
  cell_heights_.resize(num_cells);
  ground_elevation_.clear();
  ground_elevation_.resize(num_cells, std::numeric_limits<float>::quiet_NaN());

  // Initialize occupancy grid message
  occupancy_grid_.header.frame_id = map_frame_;
  occupancy_grid_.info.resolution = resolution_;
  occupancy_grid_.info.width = grid_width_;
  occupancy_grid_.info.height = grid_height_;
  occupancy_grid_.info.origin.position.x = origin_x_;
  occupancy_grid_.info.origin.position.y = origin_y_;
  occupancy_grid_.info.origin.position.z = 0.0;
  occupancy_grid_.info.origin.orientation.w = 1.0;
  occupancy_grid_.data.resize(num_cells, -1);  // Unknown

  RCLCPP_INFO(this->get_logger(), "Grid initialized:");
  RCLCPP_INFO(this->get_logger(), "  Bounds: X[%.2f, %.2f], Y[%.2f, %.2f]",
              min_pt.x, max_pt.x, min_pt.y, max_pt.y);
  RCLCPP_INFO(this->get_logger(), "  Origin: (%.2f, %.2f)", origin_x_, origin_y_);
  RCLCPP_INFO(this->get_logger(), "  Size: %.1f x %.1f m (%d x %d cells)",
              map_width_, map_height_, grid_width_, grid_height_);
}

void TerrainAnalysis::estimateGroundElevation()
{
  // Collect heights for each cell
  for (const auto & point : full_pointcloud_->points) {
    int grid_x, grid_y;
    getGridCoords(point.x, point.y, grid_x, grid_y);

    if (isInBounds(grid_x, grid_y)) {
      int index = getGridIndex(grid_x, grid_y);
      cell_heights_[index].push_back(point.z);
    }
  }

  // Estimate ground elevation using quantile method
  for (int i = 0; i < grid_width_ * grid_height_; i++) {
    if (cell_heights_[i].size() >= static_cast<size_t>(min_points_per_cell_)) {
      std::vector<float> & heights = cell_heights_[i];
      std::sort(heights.begin(), heights.end());

      // Use quantile for ground estimation (lower percentile = ground)
      int quantile_idx = static_cast<int>(quantile_z_ * heights.size());
      if (quantile_idx >= static_cast<int>(heights.size())) {
        quantile_idx = heights.size() - 1;
      }
      ground_elevation_[i] = heights[quantile_idx];
    }
  }

  // Fill gaps in ground elevation using interpolation
  // Simple neighbor averaging for cells with no ground estimate
  std::vector<float> filled_elevation = ground_elevation_;
  for (int y = 0; y < grid_height_; y++) {
    for (int x = 0; x < grid_width_; x++) {
      int idx = getGridIndex(x, y);
      if (std::isnan(ground_elevation_[idx])) {
        // Average of neighbors
        float sum = 0;
        int count = 0;
        for (int dy = -2; dy <= 2; dy++) {
          for (int dx = -2; dx <= 2; dx++) {
            if (isInBounds(x + dx, y + dy)) {
              int neighbor_idx = getGridIndex(x + dx, y + dy);
              if (!std::isnan(ground_elevation_[neighbor_idx])) {
                sum += ground_elevation_[neighbor_idx];
                count++;
              }
            }
          }
        }
        if (count > 0) {
          filled_elevation[idx] = sum / count;
        }
      }
    }
  }
  ground_elevation_ = filled_elevation;
}

void TerrainAnalysis::generateOccupancyGrid()
{
  ground_cloud_->clear();
  obstacle_cloud_->clear();

  // Reset occupancy grid
  std::fill(occupancy_grid_.data.begin(), occupancy_grid_.data.end(), -1);

  // Classify points and update occupancy grid
  for (const auto & point : full_pointcloud_->points) {
    int grid_x, grid_y;
    getGridCoords(point.x, point.y, grid_x, grid_y);

    if (!isInBounds(grid_x, grid_y)) {
      continue;
    }

    int index = getGridIndex(grid_x, grid_y);
    float ground_z = ground_elevation_[index];

    if (std::isnan(ground_z)) {
      continue;  // No ground reference
    }

    float height_above_ground = point.z - ground_z;

    if (height_above_ground < ground_clearance_) {
      // Ground point
      ground_cloud_->push_back(point);
      if (occupancy_grid_.data[index] != 100) {
        occupancy_grid_.data[index] = 0;  // Free
      }
    } else if (height_above_ground >= obstacle_height_min_ &&
               height_above_ground <= obstacle_height_max_) {
      // Obstacle point
      obstacle_cloud_->push_back(point);
      occupancy_grid_.data[index] = 100;  // Occupied
    }
  }

  // Mark cells with sufficient points as known
  for (int i = 0; i < grid_width_ * grid_height_; i++) {
    if (cell_heights_[i].size() >= static_cast<size_t>(min_points_per_cell_)) {
      if (occupancy_grid_.data[i] == -1) {
        occupancy_grid_.data[i] = 0;  // Free if no obstacles detected
      }
    }
  }
}

void TerrainAnalysis::publishResults()
{
  if (!map_generated_) {
    return;
  }

  // Publish occupancy grid
  occupancy_grid_.header.stamp = this->now();
  occupancy_pub_->publish(occupancy_grid_);

  // Publish ground cloud
  sensor_msgs::msg::PointCloud2 ground_msg;
  pcl::toROSMsg(*ground_cloud_, ground_msg);
  ground_msg.header.stamp = this->now();
  ground_msg.header.frame_id = map_frame_;
  ground_pub_->publish(ground_msg);

  // Publish obstacle cloud
  sensor_msgs::msg::PointCloud2 obstacle_msg;
  pcl::toROSMsg(*obstacle_cloud_, obstacle_msg);
  obstacle_msg.header.stamp = this->now();
  obstacle_msg.header.frame_id = map_frame_;
  obstacle_pub_->publish(obstacle_msg);
}

bool TerrainAnalysis::saveMapToFile(const std::string & filename)
{
  if (!map_generated_) {
    RCLCPP_ERROR(this->get_logger(), "Map not generated");
    return false;
  }

  // Create OpenCV image for PGM format
  cv::Mat image(grid_height_, grid_width_, CV_8UC1);

  // Convert occupancy grid to image
  // OccupancyGrid: -1=unknown, 0=free, 100=occupied
  // PGM: 0=black(occupied), 255=white(free), 205=gray(unknown)
  for (int y = 0; y < grid_height_; y++) {
    for (int x = 0; x < grid_width_; x++) {
      int index = y * grid_width_ + x;
      int8_t value = occupancy_grid_.data[index];

      uint8_t pixel;
      if (value == -1) {
        pixel = 205;  // Unknown (gray)
      } else if (value == 0) {
        pixel = 254;  // Free (white)
      } else {
        pixel = 0;    // Occupied (black)
      }

      image.at<uint8_t>(grid_height_ - 1 - y, x) = pixel;  // Flip vertically
    }
  }

  // Save PGM image
  std::string pgm_filename = filename + ".pgm";
  if (!cv::imwrite(pgm_filename, image)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to write PGM file: %s", pgm_filename.c_str());
    return false;
  }

  // Save YAML metadata
  std::string yaml_filename = filename + ".yaml";
  std::ofstream yaml_file(yaml_filename);
  if (!yaml_file.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open YAML file: %s", yaml_filename.c_str());
    return false;
  }

  yaml_file << "image: map.pgm\n";
  yaml_file << "resolution: " << resolution_ << "\n";
  yaml_file << "origin: [" << origin_x_ << ", " << origin_y_ << ", 0.0]\n";
  yaml_file << "negate: 0\n";
  yaml_file << "occupied_thresh: 0.65\n";
  yaml_file << "free_thresh: 0.196\n";

  yaml_file.close();

  RCLCPP_INFO(this->get_logger(), "Map saved to %s", filename.c_str());
  return true;
}

int TerrainAnalysis::getGridIndex(int grid_x, int grid_y) const
{
  return grid_y * grid_width_ + grid_x;
}

bool TerrainAnalysis::isInBounds(int grid_x, int grid_y) const
{
  return grid_x >= 0 && grid_x < grid_width_ && grid_y >= 0 && grid_y < grid_height_;
}

void TerrainAnalysis::getGridCoords(double x, double y, int & grid_x, int & grid_y) const
{
  grid_x = static_cast<int>(std::floor((x - origin_x_) / resolution_));
  grid_y = static_cast<int>(std::floor((y - origin_y_) / resolution_));
}

}  // namespace cub_mapping

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<cub_mapping::TerrainAnalysis>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
