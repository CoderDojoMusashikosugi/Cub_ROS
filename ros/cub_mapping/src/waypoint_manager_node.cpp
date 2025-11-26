// Copyright 2025 Cub Developer
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "cub_mapping/waypoint_manager_node.hpp"

#include <filesystem>
#include <fstream>
#include <cmath>

#include <yaml-cpp/yaml.h>
#include <visualization_msgs/msg/marker.hpp>

namespace cub_mapping
{

WaypointManagerNode::WaypointManagerNode()
: Node("waypoint_manager_node"),
  current_group_index_(0),
  data_modified_(false)
{
  // Declare parameters
  this->declare_parameter("waypoint_file", "");
  this->declare_parameter("frame_id", "map");
  this->declare_parameter("publish_rate", 10.0);
  this->declare_parameter("auto_save", false);
  this->declare_parameter("show_waypoint_numbers", true);
  this->declare_parameter("show_connections", true);

  // Get parameters
  waypoint_file_path_ = this->get_parameter("waypoint_file").as_string();
  frame_id_ = this->get_parameter("frame_id").as_string();
  publish_rate_ = this->get_parameter("publish_rate").as_double();
  auto_save_ = this->get_parameter("auto_save").as_bool();
  show_waypoint_numbers_ = this->get_parameter("show_waypoint_numbers").as_bool();
  show_connections_ = this->get_parameter("show_connections").as_bool();

  // Create services
  add_waypoint_service_ = this->create_service<srv::AddWaypoint>(
    "add_waypoint",
    std::bind(&WaypointManagerNode::addWaypointCallback, this,
      std::placeholders::_1, std::placeholders::_2));

  delete_waypoint_service_ = this->create_service<srv::DeleteWaypoint>(
    "delete_waypoint",
    std::bind(&WaypointManagerNode::deleteWaypointCallback, this,
      std::placeholders::_1, std::placeholders::_2));

  move_waypoint_service_ = this->create_service<srv::MoveWaypoint>(
    "move_waypoint",
    std::bind(&WaypointManagerNode::moveWaypointCallback, this,
      std::placeholders::_1, std::placeholders::_2));

  update_yaw_service_ = this->create_service<srv::UpdateWaypointYaw>(
    "update_waypoint_yaw",
    std::bind(&WaypointManagerNode::updateWaypointYawCallback, this,
      std::placeholders::_1, std::placeholders::_2));

  insert_waypoint_service_ = this->create_service<srv::InsertWaypoint>(
    "insert_waypoint",
    std::bind(&WaypointManagerNode::insertWaypointCallback, this,
      std::placeholders::_1, std::placeholders::_2));

  create_group_service_ = this->create_service<srv::CreateGroup>(
    "create_group",
    std::bind(&WaypointManagerNode::createGroupCallback, this,
      std::placeholders::_1, std::placeholders::_2));

  delete_group_service_ = this->create_service<srv::DeleteGroup>(
    "delete_group",
    std::bind(&WaypointManagerNode::deleteGroupCallback, this,
      std::placeholders::_1, std::placeholders::_2));

  rename_group_service_ = this->create_service<srv::RenameGroup>(
    "rename_group",
    std::bind(&WaypointManagerNode::renameGroupCallback, this,
      std::placeholders::_1, std::placeholders::_2));

  set_require_input_service_ = this->create_service<srv::SetRequireInput>(
    "set_require_input",
    std::bind(&WaypointManagerNode::setRequireInputCallback, this,
      std::placeholders::_1, std::placeholders::_2));

  save_waypoints_service_ = this->create_service<std_srvs::srv::Trigger>(
    "save_waypoints",
    std::bind(&WaypointManagerNode::saveWaypointsCallback, this,
      std::placeholders::_1, std::placeholders::_2));

  load_waypoints_service_ = this->create_service<srv::LoadWaypoints>(
    "load_waypoints",
    std::bind(&WaypointManagerNode::loadWaypointsCallback, this,
      std::placeholders::_1, std::placeholders::_2));

  get_group_list_service_ = this->create_service<srv::GetGroupList>(
    "get_group_list",
    std::bind(&WaypointManagerNode::getGroupListCallback, this,
      std::placeholders::_1, std::placeholders::_2));

  // Create publisher
  marker_array_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "waypoint_markers", 10);

  // Create timer for marker publishing
  publish_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(1.0 / publish_rate_),
    std::bind(&WaypointManagerNode::publishMarkersTimer, this));

  // Auto-load waypoint file if specified
  if (!waypoint_file_path_.empty()) {
    if (loadFromYAML(waypoint_file_path_)) {
      RCLCPP_INFO(this->get_logger(), "Auto-loaded waypoints from: %s", waypoint_file_path_.c_str());
    } else {
      RCLCPP_WARN(this->get_logger(), "Failed to auto-load waypoints from: %s",
                  waypoint_file_path_.c_str());
    }
  }

  RCLCPP_INFO(this->get_logger(), "Waypoint Manager Node initialized");
}

// Service callbacks implementation
void WaypointManagerNode::addWaypointCallback(
  const std::shared_ptr<srv::AddWaypoint::Request> request,
  std::shared_ptr<srv::AddWaypoint::Response> response)
{
  int32_t group_idx = request->group_index;

  // Use current group if -1
  if (group_idx == -1) {
    group_idx = current_group_index_;
  }

  // Validate group index
  if (group_idx < 0 || group_idx >= static_cast<int32_t>(waypoint_groups_.size())) {
    response->success = false;
    response->message = "Invalid group index: " + std::to_string(group_idx);
    RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
    return;
  }

  // Add waypoint
  Waypoint wp(request->x, request->y, request->yaw);
  waypoint_groups_[group_idx].addWaypoint(wp);
  response->waypoint_index = waypoint_groups_[group_idx].size() - 1;

  response->success = true;
  response->message = "Waypoint added successfully to group " +
                      waypoint_groups_[group_idx].group_name;

  data_modified_ = true;
  last_modification_time_ = this->now();

  RCLCPP_INFO(this->get_logger(), "Added waypoint (%.2f, %.2f, %.2f) to group %d",
              request->x, request->y, request->yaw, group_idx);

  if (auto_save_) {
    saveToYAML(waypoint_file_path_);
  }
}

void WaypointManagerNode::deleteWaypointCallback(
  const std::shared_ptr<srv::DeleteWaypoint::Request> request,
  std::shared_ptr<srv::DeleteWaypoint::Response> response)
{
  // Validate group index
  if (request->group_index < 0 ||
      request->group_index >= static_cast<int32_t>(waypoint_groups_.size())) {
    response->success = false;
    response->message = "Invalid group index: " + std::to_string(request->group_index);
    return;
  }

  auto& group = waypoint_groups_[request->group_index];

  // Validate waypoint index
  if (request->waypoint_index < 0 ||
      request->waypoint_index >= static_cast<int32_t>(group.waypoints.size())) {
    response->success = false;
    response->message = "Invalid waypoint index: " + std::to_string(request->waypoint_index);
    return;
  }

  // Delete waypoint
  group.removeWaypoint(request->waypoint_index);

  response->success = true;
  response->message = "Waypoint deleted successfully";

  data_modified_ = true;
  last_modification_time_ = this->now();

  RCLCPP_INFO(this->get_logger(), "Deleted waypoint %d from group %d",
              request->waypoint_index, request->group_index);

  if (auto_save_) {
    saveToYAML(waypoint_file_path_);
  }
}

void WaypointManagerNode::moveWaypointCallback(
  const std::shared_ptr<srv::MoveWaypoint::Request> request,
  std::shared_ptr<srv::MoveWaypoint::Response> response)
{
  // Validate indices
  if (request->group_index < 0 ||
      request->group_index >= static_cast<int32_t>(waypoint_groups_.size())) {
    response->success = false;
    response->message = "Invalid group index";
    return;
  }

  auto& group = waypoint_groups_[request->group_index];
  if (request->waypoint_index < 0 ||
      request->waypoint_index >= static_cast<int32_t>(group.waypoints.size())) {
    response->success = false;
    response->message = "Invalid waypoint index";
    return;
  }

  // Update position
  group.waypoints[request->waypoint_index].x = request->new_x;
  group.waypoints[request->waypoint_index].y = request->new_y;

  response->success = true;
  response->message = "Waypoint moved successfully";

  data_modified_ = true;
  last_modification_time_ = this->now();

  if (auto_save_) {
    saveToYAML(waypoint_file_path_);
  }
}

void WaypointManagerNode::updateWaypointYawCallback(
  const std::shared_ptr<srv::UpdateWaypointYaw::Request> request,
  std::shared_ptr<srv::UpdateWaypointYaw::Response> response)
{
  // Validate indices
  if (request->group_index < 0 ||
      request->group_index >= static_cast<int32_t>(waypoint_groups_.size())) {
    response->success = false;
    response->message = "Invalid group index";
    return;
  }

  auto& group = waypoint_groups_[request->group_index];
  if (request->waypoint_index < 0 ||
      request->waypoint_index >= static_cast<int32_t>(group.waypoints.size())) {
    response->success = false;
    response->message = "Invalid waypoint index";
    return;
  }

  // Update yaw
  group.waypoints[request->waypoint_index].yaw = request->new_yaw;

  response->success = true;
  response->message = "Waypoint yaw updated successfully";

  data_modified_ = true;
  last_modification_time_ = this->now();

  if (auto_save_) {
    saveToYAML(waypoint_file_path_);
  }
}

void WaypointManagerNode::insertWaypointCallback(
  const std::shared_ptr<srv::InsertWaypoint::Request> request,
  std::shared_ptr<srv::InsertWaypoint::Response> response)
{
  // Validate group index
  if (request->group_index < 0 ||
      request->group_index >= static_cast<int32_t>(waypoint_groups_.size())) {
    response->success = false;
    response->message = "Invalid group index";
    return;
  }

  auto& group = waypoint_groups_[request->group_index];
  Waypoint wp(request->x, request->y, request->yaw);

  // Handle append case
  if (request->insert_before_index == -1) {
    group.addWaypoint(wp);
    response->waypoint_index = group.size() - 1;
  } else {
    // Validate insert index
    if (request->insert_before_index < 0 ||
        request->insert_before_index > static_cast<int32_t>(group.waypoints.size())) {
      response->success = false;
      response->message = "Invalid insert index";
      return;
    }

    group.insertWaypoint(request->insert_before_index, wp);
    response->waypoint_index = request->insert_before_index;
  }

  response->success = true;
  response->message = "Waypoint inserted successfully";

  data_modified_ = true;
  last_modification_time_ = this->now();

  if (auto_save_) {
    saveToYAML(waypoint_file_path_);
  }
}

void WaypointManagerNode::createGroupCallback(
  const std::shared_ptr<srv::CreateGroup::Request> request,
  std::shared_ptr<srv::CreateGroup::Response> response)
{
  WaypointGroup new_group(request->group_name, request->require_input);
  waypoint_groups_.push_back(new_group);

  response->group_index = waypoint_groups_.size() - 1;
  response->success = true;
  response->message = "Group created successfully: " + request->group_name;

  data_modified_ = true;
  last_modification_time_ = this->now();

  RCLCPP_INFO(this->get_logger(), "Created group: %s (index %d)",
              request->group_name.c_str(), response->group_index);

  if (auto_save_) {
    saveToYAML(waypoint_file_path_);
  }
}

void WaypointManagerNode::deleteGroupCallback(
  const std::shared_ptr<srv::DeleteGroup::Request> request,
  std::shared_ptr<srv::DeleteGroup::Response> response)
{
  // Validate group index
  if (request->group_index < 0 ||
      request->group_index >= static_cast<int32_t>(waypoint_groups_.size())) {
    response->success = false;
    response->message = "Invalid group index";
    return;
  }

  std::string group_name = waypoint_groups_[request->group_index].group_name;
  waypoint_groups_.erase(waypoint_groups_.begin() + request->group_index);

  // Adjust current group index if necessary
  if (current_group_index_ >= static_cast<int>(waypoint_groups_.size())) {
    current_group_index_ = std::max(0, static_cast<int>(waypoint_groups_.size()) - 1);
  }

  response->success = true;
  response->message = "Group deleted successfully: " + group_name;

  data_modified_ = true;
  last_modification_time_ = this->now();

  RCLCPP_INFO(this->get_logger(), "Deleted group: %s", group_name.c_str());

  if (auto_save_) {
    saveToYAML(waypoint_file_path_);
  }
}

void WaypointManagerNode::renameGroupCallback(
  const std::shared_ptr<srv::RenameGroup::Request> request,
  std::shared_ptr<srv::RenameGroup::Response> response)
{
  // Validate group index
  if (request->group_index < 0 ||
      request->group_index >= static_cast<int32_t>(waypoint_groups_.size())) {
    response->success = false;
    response->message = "Invalid group index";
    return;
  }

  std::string old_name = waypoint_groups_[request->group_index].group_name;
  waypoint_groups_[request->group_index].group_name = request->new_name;

  response->success = true;
  response->message = "Group renamed from '" + old_name + "' to '" + request->new_name + "'";

  data_modified_ = true;
  last_modification_time_ = this->now();

  if (auto_save_) {
    saveToYAML(waypoint_file_path_);
  }
}

void WaypointManagerNode::setRequireInputCallback(
  const std::shared_ptr<srv::SetRequireInput::Request> request,
  std::shared_ptr<srv::SetRequireInput::Response> response)
{
  // Validate group index
  if (request->group_index < 0 ||
      request->group_index >= static_cast<int32_t>(waypoint_groups_.size())) {
    response->success = false;
    response->message = "Invalid group index";
    return;
  }

  waypoint_groups_[request->group_index].require_input = request->require_input;

  response->success = true;
  response->message = "require_input flag updated";

  data_modified_ = true;
  last_modification_time_ = this->now();

  if (auto_save_) {
    saveToYAML(waypoint_file_path_);
  }
}

void WaypointManagerNode::saveWaypointsCallback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  if (waypoint_file_path_.empty()) {
    response->success = false;
    response->message = "No waypoint file path configured";
    return;
  }

  if (saveToYAML(waypoint_file_path_)) {
    response->success = true;
    response->message = "Waypoints saved successfully to " + waypoint_file_path_;
    data_modified_ = false;
  } else {
    response->success = false;
    response->message = "Failed to save waypoints";
  }
}

void WaypointManagerNode::loadWaypointsCallback(
  const std::shared_ptr<srv::LoadWaypoints::Request> request,
  std::shared_ptr<srv::LoadWaypoints::Response> response)
{
  if (loadFromYAML(request->filepath)) {
    response->success = true;
    response->num_groups_loaded = waypoint_groups_.size();

    int total_waypoints = 0;
    for (const auto& group : waypoint_groups_) {
      total_waypoints += group.size();
    }
    response->num_waypoints_loaded = total_waypoints;

    response->message = "Loaded " + std::to_string(response->num_groups_loaded) +
                       " groups with " + std::to_string(total_waypoints) + " waypoints";

    waypoint_file_path_ = request->filepath;
    data_modified_ = false;
  } else {
    response->success = false;
    response->message = "Failed to load waypoints from " + request->filepath;
  }
}

void WaypointManagerNode::getGroupListCallback(
  const std::shared_ptr<srv::GetGroupList::Request> /*request*/,
  std::shared_ptr<srv::GetGroupList::Response> response)
{
  response->group_names.clear();
  response->require_input_flags.clear();

  for (const auto& group : waypoint_groups_) {
    response->group_names.push_back(group.group_name);
    response->require_input_flags.push_back(group.require_input);
  }
}

// YAML I/O implementation
bool WaypointManagerNode::loadFromYAML(const std::string& filepath)
{
  try {
    YAML::Node config = YAML::LoadFile(filepath);

    if (!config["waypoint_groups"]) {
      RCLCPP_ERROR(this->get_logger(), "No 'waypoint_groups' key in YAML file");
      return false;
    }

    waypoint_groups_.clear();

    for (const auto& group_node : config["waypoint_groups"]) {
      WaypointGroup group;
      group.group_name = group_node["group_name"].as<std::string>();
      group.require_input = group_node["require_input"].as<bool>();

      if (group_node["waypoints"]) {
        for (const auto& wp_node : group_node["waypoints"]) {
          Waypoint wp;
          wp.x = wp_node["x"].as<double>();
          wp.y = wp_node["y"].as<double>();
          wp.yaw = wp_node["yaw"].as<double>();
          group.waypoints.push_back(wp);
        }
      }

      waypoint_groups_.push_back(group);
    }

    RCLCPP_INFO(this->get_logger(), "Loaded %zu groups from %s",
                waypoint_groups_.size(), filepath.c_str());

    // Set current group to first group if available
    if (!waypoint_groups_.empty()) {
      current_group_index_ = 0;
    }

    return true;

  } catch (const YAML::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "YAML error: %s", e.what());
    return false;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Error loading YAML: %s", e.what());
    return false;
  }
}

bool WaypointManagerNode::saveToYAML(const std::string& filepath)
{
  try {
    // Create backup of existing file
    if (std::filesystem::exists(filepath)) {
      std::string backup_path = filepath + ".bak";
      std::filesystem::copy_file(
        filepath, backup_path,
        std::filesystem::copy_options::overwrite_existing);
      RCLCPP_INFO(this->get_logger(), "Backup created: %s", backup_path.c_str());
    }

    // Build YAML structure
    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "waypoint_groups";
    out << YAML::Value << YAML::BeginSeq;

    for (const auto& group : waypoint_groups_) {
      out << YAML::BeginMap;
      out << YAML::Key << "group_name" << YAML::Value << group.group_name;
      out << YAML::Key << "require_input" << YAML::Value << group.require_input;
      out << YAML::Key << "waypoints" << YAML::Value << YAML::BeginSeq;

      for (const auto& wp : group.waypoints) {
        out << YAML::BeginMap;
        out << YAML::Key << "x" << YAML::Value << wp.x;
        out << YAML::Key << "y" << YAML::Value << wp.y;
        out << YAML::Key << "yaw" << YAML::Value << wp.yaw;
        out << YAML::EndMap;
      }

      out << YAML::EndSeq;  // waypoints
      out << YAML::EndMap;  // group
    }

    out << YAML::EndSeq;  // waypoint_groups
    out << YAML::EndMap;  // root

    // Write to file
    std::ofstream fout(filepath);
    if (!fout.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open file: %s", filepath.c_str());
      return false;
    }

    fout << out.c_str();
    fout.close();

    RCLCPP_INFO(this->get_logger(), "Saved %zu groups to %s",
                waypoint_groups_.size(), filepath.c_str());

    return true;

  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Error saving YAML: %s", e.what());
    return false;
  }
}

// Visualization implementation
void WaypointManagerNode::publishMarkersTimer()
{
  publishWaypointMarkers();
}

void WaypointManagerNode::publishWaypointMarkers()
{
  visualization_msgs::msg::MarkerArray marker_array;

  // Delete all previous markers
  visualization_msgs::msg::Marker delete_marker;
  delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(delete_marker);

  int marker_id = 0;

  for (size_t group_idx = 0; group_idx < waypoint_groups_.size(); ++group_idx) {
    const auto& group = waypoint_groups_[group_idx];
    auto color = getGroupColor(group_idx);

    // Create arrow markers for waypoints
    for (size_t wp_idx = 0; wp_idx < group.waypoints.size(); ++wp_idx) {
      const auto& wp = group.waypoints[wp_idx];

      visualization_msgs::msg::Marker arrow_marker;
      arrow_marker.header.frame_id = frame_id_;
      arrow_marker.header.stamp = this->now();
      arrow_marker.ns = "waypoint_" + std::to_string(group_idx) + "_" + std::to_string(wp_idx);
      arrow_marker.id = marker_id++;
      arrow_marker.type = visualization_msgs::msg::Marker::ARROW;
      arrow_marker.action = visualization_msgs::msg::Marker::ADD;

      arrow_marker.pose.position.x = wp.x;
      arrow_marker.pose.position.y = wp.y;
      arrow_marker.pose.position.z = 0.0;

      // Convert yaw to quaternion
      double half_yaw = wp.yaw * 0.5;
      arrow_marker.pose.orientation.x = 0.0;
      arrow_marker.pose.orientation.y = 0.0;
      arrow_marker.pose.orientation.z = std::sin(half_yaw);
      arrow_marker.pose.orientation.w = std::cos(half_yaw);

      arrow_marker.scale.x = 0.5;  // Length
      arrow_marker.scale.y = 0.1;  // Width
      arrow_marker.scale.z = 0.1;  // Height

      arrow_marker.color.r = color[0];
      arrow_marker.color.g = color[1];
      arrow_marker.color.b = color[2];
      arrow_marker.color.a = 1.0;

      marker_array.markers.push_back(arrow_marker);

      // Add text marker for waypoint number if enabled
      if (show_waypoint_numbers_) {
        visualization_msgs::msg::Marker text_marker;
        text_marker.header.frame_id = frame_id_;
        text_marker.header.stamp = this->now();
        text_marker.ns = "waypoint_numbers";
        text_marker.id = marker_id++;
        text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::msg::Marker::ADD;

        text_marker.pose.position.x = wp.x;
        text_marker.pose.position.y = wp.y;
        text_marker.pose.position.z = 0.5;  // Above the arrow

        text_marker.scale.z = 0.2;  // Text size

        text_marker.color.r = 1.0;
        text_marker.color.g = 1.0;
        text_marker.color.b = 1.0;
        text_marker.color.a = 1.0;

        text_marker.text = std::to_string(wp_idx);

        marker_array.markers.push_back(text_marker);
      }
    }

    // Add line strip connecting waypoints if enabled
    if (show_connections_ && group.waypoints.size() > 1) {
      visualization_msgs::msg::Marker line_marker;
      line_marker.header.frame_id = frame_id_;
      line_marker.header.stamp = this->now();
      line_marker.ns = "waypoint_connections";
      line_marker.id = marker_id++;
      line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      line_marker.action = visualization_msgs::msg::Marker::ADD;

      line_marker.scale.x = 0.02;  // Line width

      line_marker.color.r = color[0] * 0.7;  // Lighter shade
      line_marker.color.g = color[1] * 0.7;
      line_marker.color.b = color[2] * 0.7;
      line_marker.color.a = 0.5;

      for (const auto& wp : group.waypoints) {
        geometry_msgs::msg::Point p;
        p.x = wp.x;
        p.y = wp.y;
        p.z = 0.0;
        line_marker.points.push_back(p);
      }

      marker_array.markers.push_back(line_marker);
    }
  }

  marker_array_pub_->publish(marker_array);
}

std::array<float, 3> WaypointManagerNode::getGroupColor(size_t group_index)
{
  // Generate rainbow colors based on group index
  float hue = (group_index * 137.5) / 360.0;  // Golden angle for good distribution
  hue = hue - std::floor(hue);  // Keep in [0, 1] range

  // Convert HSV to RGB (S=1, V=1)
  float h = hue * 6.0f;
  float c = 1.0f;
  float x = c * (1.0f - std::abs(std::fmod(h, 2.0f) - 1.0f));

  std::array<float, 3> rgb;
  if (h < 1.0f) {
    rgb = {c, x, 0.0f};
  } else if (h < 2.0f) {
    rgb = {x, c, 0.0f};
  } else if (h < 3.0f) {
    rgb = {0.0f, c, x};
  } else if (h < 4.0f) {
    rgb = {0.0f, x, c};
  } else if (h < 5.0f) {
    rgb = {x, 0.0f, c};
  } else {
    rgb = {c, 0.0f, x};
  }

  return rgb;
}

}  // namespace cub_mapping

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<cub_mapping::WaypointManagerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
