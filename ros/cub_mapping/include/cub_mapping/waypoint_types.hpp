// Copyright 2025 Cub Developer
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#ifndef CUB_MAPPING__WAYPOINT_TYPES_HPP_
#define CUB_MAPPING__WAYPOINT_TYPES_HPP_

#include <string>
#include <vector>

namespace cub_mapping
{

/**
 * @brief Represents a single waypoint with position and orientation
 */
struct Waypoint {
  double x;    ///< X coordinate in map frame (meters)
  double y;    ///< Y coordinate in map frame (meters)
  double yaw;  ///< Orientation in radians

  Waypoint() : x(0.0), y(0.0), yaw(0.0) {}

  Waypoint(double x_, double y_, double yaw_)
  : x(x_), y(y_), yaw(yaw_) {}
};

/**
 * @brief Represents a group of waypoints with metadata
 */
struct WaypointGroup {
  std::string group_name;      ///< Name of the waypoint group
  bool require_input;          ///< Whether user input is required before navigating this group
  std::vector<Waypoint> waypoints;  ///< List of waypoints in this group

  WaypointGroup() : require_input(false) {}

  WaypointGroup(const std::string& name, bool require_input_)
  : group_name(name), require_input(require_input_) {}

  /**
   * @brief Check if the group is empty
   * @return True if the group has no waypoints
   */
  bool isEmpty() const { return waypoints.empty(); }

  /**
   * @brief Get the number of waypoints in the group
   * @return Number of waypoints
   */
  size_t size() const { return waypoints.size(); }

  /**
   * @brief Add a waypoint to the end of the group
   * @param wp Waypoint to add
   */
  void addWaypoint(const Waypoint& wp) { waypoints.push_back(wp); }

  /**
   * @brief Remove a waypoint at the specified index
   * @param index Index of waypoint to remove
   * @return True if waypoint was removed successfully
   */
  bool removeWaypoint(size_t index)
  {
    if (index >= waypoints.size()) {
      return false;
    }
    waypoints.erase(waypoints.begin() + index);
    return true;
  }

  /**
   * @brief Insert a waypoint at the specified index
   * @param index Index where waypoint should be inserted
   * @param wp Waypoint to insert
   * @return True if waypoint was inserted successfully
   */
  bool insertWaypoint(size_t index, const Waypoint& wp)
  {
    if (index > waypoints.size()) {
      return false;
    }
    waypoints.insert(waypoints.begin() + index, wp);
    return true;
  }
};

}  // namespace cub_mapping

#endif  // CUB_MAPPING__WAYPOINT_TYPES_HPP_
