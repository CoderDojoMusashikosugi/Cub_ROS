// Copyright 2025 Cub Developer
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "cub_mapping/map_converter_node.hpp"

#include <fstream>
#include <iomanip>
#include <cmath>
#include <ctime>
#include <sys/stat.h>

#include <opencv2/opencv.hpp>
#include <pcl/filters/crop_box.h>
#include <pcl/common/common.h>

namespace cub_mapping
{

MapConverterNode::MapConverterNode()
: Node("map_converter_node"),
  full_pointcloud_(new pcl::PointCloud<pcl::PointXYZ>()),
  map_initialized_(false),
  region_counter_(0),
  last_pointcloud_time_(this->now())
{
  // Declare parameters with default values
  this->declare_parameter("resolution", 0.05);
  this->declare_parameter("map_width", 50.0);
  this->declare_parameter("map_height", 50.0);
  this->declare_parameter("origin_x", -25.0);
  this->declare_parameter("origin_y", -25.0);
  this->declare_parameter("map_frame", "map");
  this->declare_parameter("pointcloud_topic", "/cloud_pcd");
  this->declare_parameter("output_directory", "/home/cub/maps");
  this->declare_parameter("publish_rate", 1.0);
  this->declare_parameter("slice_height", 0.0);
  this->declare_parameter("tf_publish_rate", 10.0);

  // Get parameters
  resolution_ = this->get_parameter("resolution").as_double();
  map_width_ = this->get_parameter("map_width").as_double();
  map_height_ = this->get_parameter("map_height").as_double();
  origin_x_ = this->get_parameter("origin_x").as_double();
  origin_y_ = this->get_parameter("origin_y").as_double();
  map_frame_ = this->get_parameter("map_frame").as_string();
  pointcloud_topic_ = this->get_parameter("pointcloud_topic").as_string();
  output_directory_ = this->get_parameter("output_directory").as_string();
  publish_rate_ = this->get_parameter("publish_rate").as_double();
  slice_height_ = this->get_parameter("slice_height").as_double();
  tf_publish_rate_ = this->get_parameter("tf_publish_rate").as_double();

  // Note: occupancy grid will be initialized when first point cloud is received

  // Create TF broadcaster
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  // Create services
  add_region_service_ = this->create_service<cub_mapping::srv::AddRegion>(
    "add_region",
    std::bind(&MapConverterNode::addRegionCallback, this,
              std::placeholders::_1, std::placeholders::_2));

  save_map_service_ = this->create_service<std_srvs::srv::Trigger>(
    "save_map",
    std::bind(&MapConverterNode::saveMapCallback, this,
              std::placeholders::_1, std::placeholders::_2));

  clear_map_service_ = this->create_service<std_srvs::srv::Trigger>(
    "clear_map",
    std::bind(&MapConverterNode::clearMapCallback, this,
              std::placeholders::_1, std::placeholders::_2));

  // Create subscriptions
  pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    pointcloud_topic_, 10,
    std::bind(&MapConverterNode::pointCloudCallback, this, std::placeholders::_1));

  slice_height_sub_ = this->create_subscription<std_msgs::msg::Float64>(
    "map_slice_height", 10,
    std::bind(&MapConverterNode::sliceHeightCallback, this, std::placeholders::_1));

  // Create publishers
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "occupancy_grid", 10);

  region_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
    "region_markers", 10);

  // Create timer for periodic map publishing
  auto timer_period = std::chrono::duration<double>(1.0 / publish_rate_);
  map_publish_timer_ = this->create_wall_timer(
    timer_period,
    std::bind(&MapConverterNode::publishMap, this));

  // Create timer for TF broadcasting (higher frequency than map publishing)
  auto tf_timer_period = std::chrono::duration<double>(1.0 / tf_publish_rate_);
  tf_publish_timer_ = this->create_wall_timer(
    tf_timer_period,
    std::bind(&MapConverterNode::publishMapSliceTransform, this));

  RCLCPP_INFO(this->get_logger(), "Map Converter Node initialized");
  RCLCPP_INFO(this->get_logger(), "  Resolution: %.3f m/cell", resolution_);
  RCLCPP_INFO(this->get_logger(), "  Frame: %s", map_frame_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Subscribing to: %s", pointcloud_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Map size will be auto-configured from point cloud bounds");
  RCLCPP_INFO(this->get_logger(), "  Initial slice height: %.2f m", slice_height_);
  RCLCPP_INFO(this->get_logger(), "  TF broadcast rate: %.1f Hz", tf_publish_rate_);
}

void MapConverterNode::initializeOccupancyGrid()
{
  // Calculate grid dimensions
  int width_cells = static_cast<int>(std::ceil(map_width_ / resolution_));
  int height_cells = static_cast<int>(std::ceil(map_height_ / resolution_));

  // Initialize occupancy grid message
  // Use map_slice frame so 2D map follows the slice plane
  occupancy_grid_.header.frame_id = "map_slice";
  occupancy_grid_.info.resolution = resolution_;
  occupancy_grid_.info.width = width_cells;
  occupancy_grid_.info.height = height_cells;
  occupancy_grid_.info.origin.position.x = origin_x_;
  occupancy_grid_.info.origin.position.y = origin_y_;
  occupancy_grid_.info.origin.position.z = 0.0;
  occupancy_grid_.info.origin.orientation.w = 1.0;

  // Initialize all cells as unknown (-1)
  occupancy_grid_.data.resize(width_cells * height_cells, -1);

  map_initialized_ = true;

  RCLCPP_INFO(this->get_logger(), "Occupancy grid initialized: %d x %d cells",
              width_cells, height_cells);
}

void MapConverterNode::pointCloudCallback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // Convert ROS message to PCL point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*msg, *cloud);

  // Store the latest point cloud
  *full_pointcloud_ = *cloud;
  last_pointcloud_time_ = msg->header.stamp;

  // If this is the first point cloud, compute bounding box and initialize map
  if (!map_initialized_ && !cloud->empty()) {
    // Compute bounding box
    pcl::PointXYZ min_pt, max_pt;
    pcl::getMinMax3D(*cloud, min_pt, max_pt);

    // Calculate map dimensions with some padding (10% margin)
    double margin_factor = 1.1;
    double width = (max_pt.x - min_pt.x) * margin_factor;
    double height = (max_pt.y - min_pt.y) * margin_factor;

    // Calculate map center
    double center_x = (max_pt.x + min_pt.x) / 2.0;
    double center_y = (max_pt.y + min_pt.y) / 2.0;

    // Update map parameters
    map_width_ = width;
    map_height_ = height;
    origin_x_ = center_x - width / 2.0;
    origin_y_ = center_y - height / 2.0;

    RCLCPP_INFO(this->get_logger(), "Auto-configured map from point cloud:");
    RCLCPP_INFO(this->get_logger(), "  Point cloud bounds: X[%.2f, %.2f], Y[%.2f, %.2f]",
                min_pt.x, max_pt.x, min_pt.y, max_pt.y);
    RCLCPP_INFO(this->get_logger(), "  Map size: %.1f x %.1f m", map_width_, map_height_);
    RCLCPP_INFO(this->get_logger(), "  Map origin: (%.2f, %.2f)", origin_x_, origin_y_);

    // Initialize the occupancy grid with new parameters
    initializeOccupancyGrid();
  }

  RCLCPP_DEBUG(this->get_logger(), "Received point cloud with %zu points", cloud->size());
}

void MapConverterNode::sliceHeightCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
  slice_height_ = msg->data;
  RCLCPP_DEBUG(this->get_logger(), "Slice height updated to: %.2f m", slice_height_);
}

void MapConverterNode::addRegionCallback(
  const std::shared_ptr<cub_mapping::srv::AddRegion::Request> request,
  std::shared_ptr<cub_mapping::srv::AddRegion::Response> response)
{
  RCLCPP_INFO(this->get_logger(),
              "Adding region: center=(%.2f, %.2f, %.2f), width=%.2f, range=%.2f",
              request->center_x, request->center_y, request->center_z,
              request->width, request->height_range);

  if (full_pointcloud_->empty()) {
    RCLCPP_WARN(this->get_logger(), "No point cloud data available yet");
    response->success = false;
    response->message = "No point cloud data available";
    response->points_processed = 0;
    return;
  }

  // Process the region
  int points_processed = processRegion(
    request->center_x, request->center_y, request->center_z,
    request->width, request->height_range);

  // Publish region marker for visualization
  publishRegionMarker(
    request->center_x, request->center_y, request->center_z,
    request->width, request->height_range, region_counter_);

  region_counter_++;

  response->success = true;
  response->message = "Region processed successfully";
  response->points_processed = points_processed;

  RCLCPP_INFO(this->get_logger(), "Processed %d points from region", points_processed);
}

int MapConverterNode::processRegion(
  double center_x, double center_y, double center_z,
  double width, double height_range)
{
  // Create a crop box filter to extract points in the specified region
  pcl::CropBox<pcl::PointXYZ> crop_filter;
  crop_filter.setInputCloud(full_pointcloud_);

  // Set the bounding box
  Eigen::Vector4f min_point(
    center_x - width / 2.0,
    center_y - width / 2.0,
    center_z - height_range / 2.0,
    1.0);

  Eigen::Vector4f max_point(
    center_x + width / 2.0,
    center_y + width / 2.0,
    center_z + height_range / 2.0,
    1.0);

  crop_filter.setMin(min_point);
  crop_filter.setMax(max_point);

  // Filter the point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_region(new pcl::PointCloud<pcl::PointXYZ>());
  crop_filter.filter(*cloud_region);

  // Update the occupancy grid with the filtered points
  updateOccupancyGrid(cloud_region, center_x, center_y, width);

  return cloud_region->size();
}

void MapConverterNode::updateOccupancyGrid(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_region,
  double center_x, double center_y, double width)
{
  if (!map_initialized_) {
    return;
  }

  // Calculate grid bounds for the region
  int region_min_x = static_cast<int>(
    std::floor((center_x - width / 2.0 - origin_x_) / resolution_));
  int region_max_x = static_cast<int>(
    std::floor((center_x + width / 2.0 - origin_x_) / resolution_));
  int region_min_y = static_cast<int>(
    std::floor((center_y - width / 2.0 - origin_y_) / resolution_));
  int region_max_y = static_cast<int>(
    std::floor((center_y + width / 2.0 - origin_y_) / resolution_));

  // Clamp to grid bounds
  region_min_x = std::max(0, region_min_x);
  region_max_x = std::min(static_cast<int>(occupancy_grid_.info.width) - 1, region_max_x);
  region_min_y = std::max(0, region_min_y);
  region_max_y = std::min(static_cast<int>(occupancy_grid_.info.height) - 1, region_max_y);

  // First pass: Mark all cells in the region as free (0)
  for (int grid_y = region_min_y; grid_y <= region_max_y; ++grid_y) {
    for (int grid_x = region_min_x; grid_x <= region_max_x; ++grid_x) {
      int index = grid_y * occupancy_grid_.info.width + grid_x;
      occupancy_grid_.data[index] = 0;  // Free space
    }
  }

  // Second pass: Mark cells with points as occupied (100)
  for (const auto & point : cloud_region->points) {
    // Calculate grid coordinates
    int grid_x = static_cast<int>(
      std::floor((point.x - origin_x_) / resolution_));
    int grid_y = static_cast<int>(
      std::floor((point.y - origin_y_) / resolution_));

    // Check if within bounds
    if (grid_x >= 0 && grid_x < static_cast<int>(occupancy_grid_.info.width) &&
        grid_y >= 0 && grid_y < static_cast<int>(occupancy_grid_.info.height))
    {
      // Calculate index in 1D array
      int index = grid_y * occupancy_grid_.info.width + grid_x;

      // Mark as occupied (100 = fully occupied)
      occupancy_grid_.data[index] = 100;
    }
  }
}

void MapConverterNode::publishMap()
{
  if (!map_initialized_) {
    return;
  }

  occupancy_grid_.header.stamp = this->now();
  map_pub_->publish(occupancy_grid_);
}

void MapConverterNode::publishMapSliceTransform()
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = this->now();
  transform.header.frame_id = map_frame_;
  transform.child_frame_id = "map_slice";

  transform.transform.translation.x = 0.0;
  transform.transform.translation.y = 0.0;
  transform.transform.translation.z = slice_height_;

  transform.transform.rotation.x = 0.0;
  transform.transform.rotation.y = 0.0;
  transform.transform.rotation.z = 0.0;
  transform.transform.rotation.w = 1.0;

  tf_broadcaster_->sendTransform(transform);
}

void MapConverterNode::publishRegionMarker(
  double center_x, double center_y, double center_z,
  double width, double height_range, int region_id)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = map_frame_;
  marker.header.stamp = this->now();
  marker.ns = "region_boxes";
  marker.id = region_id;
  marker.type = visualization_msgs::msg::Marker::CUBE;
  marker.action = visualization_msgs::msg::Marker::ADD;

  marker.pose.position.x = center_x;
  marker.pose.position.y = center_y;
  marker.pose.position.z = center_z;
  marker.pose.orientation.w = 1.0;

  marker.scale.x = width;
  marker.scale.y = width;
  marker.scale.z = height_range;

  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.color.a = 0.3;

  marker.lifetime = rclcpp::Duration::from_seconds(0);  // Never expire

  region_marker_pub_->publish(marker);
}

void MapConverterNode::saveMapCallback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  // Generate directory name with timestamp (YYYYMMDD_HHMMSS format)
  auto now = this->now();
  std::time_t time_t_seconds = static_cast<std::time_t>(now.seconds());
  std::tm * tm_local = std::localtime(&time_t_seconds);

  std::stringstream dir_ss;
  dir_ss << output_directory_ << "/"
         << std::setfill('0')
         << std::setw(4) << (tm_local->tm_year + 1900)
         << std::setw(2) << (tm_local->tm_mon + 1)
         << std::setw(2) << tm_local->tm_mday
         << "_"
         << std::setw(2) << tm_local->tm_hour
         << std::setw(2) << tm_local->tm_min
         << std::setw(2) << tm_local->tm_sec;
  std::string map_directory = dir_ss.str();

  // Create directory
  if (mkdir(map_directory.c_str(), 0755) != 0 && errno != EEXIST) {
    response->success = false;
    response->message = "Failed to create directory: " + map_directory;
    RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
    return;
  }

  // Save map files
  std::string base_filename = map_directory + "/map";
  bool success = saveMapToFile(base_filename);

  if (success) {
    response->success = true;
    response->message = "Map saved to " + base_filename + ".pgm/.yaml";
    RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
  } else {
    response->success = false;
    response->message = "Failed to save map";
    RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
  }
}

bool MapConverterNode::saveMapToFile(const std::string & filename)
{
  if (!map_initialized_) {
    RCLCPP_ERROR(this->get_logger(), "Map not initialized");
    return false;
  }

  int width = occupancy_grid_.info.width;
  int height = occupancy_grid_.info.height;

  // Create OpenCV image for PGM format
  cv::Mat image(height, width, CV_8UC1);

  // Convert occupancy grid to image
  // OccupancyGrid: -1=unknown, 0=free, 100=occupied
  // PGM: 0=black(occupied), 255=white(free), 205=gray(unknown)
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      int index = y * width + x;
      int8_t value = occupancy_grid_.data[index];

      uint8_t pixel;
      if (value == -1) {
        pixel = 205;  // Unknown (gray)
      } else if (value == 0) {
        pixel = 254;  // Free (white)
      } else {
        // Occupied: map 1-100 to 0-253
        pixel = 254 - static_cast<uint8_t>((value * 254) / 100);
      }

      image.at<uint8_t>(height - 1 - y, x) = pixel;  // Flip vertically
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

  // Use just "map.pgm" as the image filename (same directory as yaml)
  yaml_file << "image: map.pgm\n";
  yaml_file << "resolution: " << resolution_ << "\n";
  yaml_file << "origin: ["
            << origin_x_ << ", "
            << origin_y_ << ", "
            << 0.0 << "]\n";
  yaml_file << "negate: 0\n";
  yaml_file << "occupied_thresh: 0.65\n";
  yaml_file << "free_thresh: 0.196\n";

  yaml_file.close();

  return true;
}

void MapConverterNode::clearMapCallback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  // Reset all cells to unknown
  std::fill(occupancy_grid_.data.begin(), occupancy_grid_.data.end(), -1);
  region_counter_ = 0;

  response->success = true;
  response->message = "Map cleared successfully";

  RCLCPP_INFO(this->get_logger(), "Map cleared");
}

}  // namespace cub_mapping

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<cub_mapping::MapConverterNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
