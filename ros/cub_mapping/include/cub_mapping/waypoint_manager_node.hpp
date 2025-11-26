// Copyright 2025 Cub Developer
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#ifndef CUB_MAPPING__WAYPOINT_MANAGER_NODE_HPP_
#define CUB_MAPPING__WAYPOINT_MANAGER_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "cub_mapping/waypoint_types.hpp"
#include "cub_mapping/srv/add_waypoint.hpp"
#include "cub_mapping/srv/delete_waypoint.hpp"
#include "cub_mapping/srv/move_waypoint.hpp"
#include "cub_mapping/srv/update_waypoint_yaw.hpp"
#include "cub_mapping/srv/insert_waypoint.hpp"
#include "cub_mapping/srv/create_group.hpp"
#include "cub_mapping/srv/delete_group.hpp"
#include "cub_mapping/srv/rename_group.hpp"
#include "cub_mapping/srv/set_require_input.hpp"
#include "cub_mapping/srv/load_waypoints.hpp"
#include "cub_mapping/srv/get_group_list.hpp"

namespace cub_mapping
{

/**
 * @brief Backend node for managing waypoint groups and persistence
 */
class WaypointManagerNode : public rclcpp::Node
{
public:
  WaypointManagerNode();
  virtual ~WaypointManagerNode() = default;

private:
  // Service callbacks
  void addWaypointCallback(
    const std::shared_ptr<srv::AddWaypoint::Request> request,
    std::shared_ptr<srv::AddWaypoint::Response> response);

  void deleteWaypointCallback(
    const std::shared_ptr<srv::DeleteWaypoint::Request> request,
    std::shared_ptr<srv::DeleteWaypoint::Response> response);

  void moveWaypointCallback(
    const std::shared_ptr<srv::MoveWaypoint::Request> request,
    std::shared_ptr<srv::MoveWaypoint::Response> response);

  void updateWaypointYawCallback(
    const std::shared_ptr<srv::UpdateWaypointYaw::Request> request,
    std::shared_ptr<srv::UpdateWaypointYaw::Response> response);

  void insertWaypointCallback(
    const std::shared_ptr<srv::InsertWaypoint::Request> request,
    std::shared_ptr<srv::InsertWaypoint::Response> response);

  void createGroupCallback(
    const std::shared_ptr<srv::CreateGroup::Request> request,
    std::shared_ptr<srv::CreateGroup::Response> response);

  void deleteGroupCallback(
    const std::shared_ptr<srv::DeleteGroup::Request> request,
    std::shared_ptr<srv::DeleteGroup::Response> response);

  void renameGroupCallback(
    const std::shared_ptr<srv::RenameGroup::Request> request,
    std::shared_ptr<srv::RenameGroup::Response> response);

  void setRequireInputCallback(
    const std::shared_ptr<srv::SetRequireInput::Request> request,
    std::shared_ptr<srv::SetRequireInput::Response> response);

  void saveWaypointsCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void loadWaypointsCallback(
    const std::shared_ptr<srv::LoadWaypoints::Request> request,
    std::shared_ptr<srv::LoadWaypoints::Response> response);

  void getGroupListCallback(
    const std::shared_ptr<srv::GetGroupList::Request> request,
    std::shared_ptr<srv::GetGroupList::Response> response);

  // YAML I/O
  bool loadFromYAML(const std::string& filepath);
  bool saveToYAML(const std::string& filepath);

  // Visualization
  void publishWaypointMarkers();
  void publishMarkersTimer();

  // Utility
  std::array<float, 3> getGroupColor(size_t group_index);

  // ROS2 communication
  rclcpp::Service<srv::AddWaypoint>::SharedPtr add_waypoint_service_;
  rclcpp::Service<srv::DeleteWaypoint>::SharedPtr delete_waypoint_service_;
  rclcpp::Service<srv::MoveWaypoint>::SharedPtr move_waypoint_service_;
  rclcpp::Service<srv::UpdateWaypointYaw>::SharedPtr update_yaw_service_;
  rclcpp::Service<srv::InsertWaypoint>::SharedPtr insert_waypoint_service_;
  rclcpp::Service<srv::CreateGroup>::SharedPtr create_group_service_;
  rclcpp::Service<srv::DeleteGroup>::SharedPtr delete_group_service_;
  rclcpp::Service<srv::RenameGroup>::SharedPtr rename_group_service_;
  rclcpp::Service<srv::SetRequireInput>::SharedPtr set_require_input_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_waypoints_service_;
  rclcpp::Service<srv::LoadWaypoints>::SharedPtr load_waypoints_service_;
  rclcpp::Service<srv::GetGroupList>::SharedPtr get_group_list_service_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  // Data storage
  std::vector<WaypointGroup> waypoint_groups_;
  int current_group_index_;

  // Parameters
  std::string waypoint_file_path_;
  std::string frame_id_;
  double publish_rate_;
  bool auto_save_;
  bool show_waypoint_numbers_;
  bool show_connections_;

  // State tracking
  bool data_modified_;
  rclcpp::Time last_modification_time_;
};

}  // namespace cub_mapping

#endif  // CUB_MAPPING__WAYPOINT_MANAGER_NODE_HPP_
