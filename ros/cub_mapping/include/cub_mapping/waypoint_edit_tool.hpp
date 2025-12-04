// Copyright 2025 Cub Developer
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#ifndef CUB_MAPPING__WAYPOINT_EDIT_TOOL_HPP_
#define CUB_MAPPING__WAYPOINT_EDIT_TOOL_HPP_

#include <memory>
#include <string>

#include <QObject>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/tool.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/viewport_mouse_event.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/properties/string_property.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_rendering/objects/arrow.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "cub_mapping/srv/add_waypoint.hpp"
#include "cub_mapping/srv/create_group.hpp"
#include "cub_mapping/srv/delete_waypoint.hpp"
#include "cub_mapping/srv/get_group_list.hpp"
#include "cub_mapping/srv/insert_waypoint.hpp"

#include <OgreVector3.h>

namespace Ogre
{
class SceneNode;
}

namespace cub_mapping
{

/**
 * @brief RViz2 tool for interactive waypoint editing
 *
 * This tool allows users to place and edit waypoints for navigation:
 * - Mouse movement: Position the waypoint cursor
 * - Mouse scroll: Adjust waypoint orientation (yaw)
 * - Left click: Place waypoint at end of current group
 * - Shift + Left click: Insert waypoint after nearest existing waypoint
 * - Press 'w' to activate tool
 */
class WaypointEditTool : public rviz_common::Tool
{
  Q_OBJECT

public:
  WaypointEditTool();
  virtual ~WaypointEditTool();

  void onInitialize() override;
  void activate() override;
  void deactivate() override;

  int processMouseEvent(rviz_common::ViewportMouseEvent & event) override;
  int processKeyEvent(QKeyEvent * event, rviz_common::RenderPanel * panel) override;

private Q_SLOTS:
  void updateProperties();
  void updateCurrentGroup();

private:
  // Editing modes
  enum EditMode {
    MODE_ADD_WAYPOINT,    // Default: Click to add waypoints
    MODE_DELETE_WAYPOINT, // Click to delete waypoint
    MODE_SELECT,          // Click to select waypoint
    MODE_MOVE_POSITION,   // Drag to move waypoint
    MODE_ROTATE_ORIENTATION  // Drag/scroll to rotate waypoint
  };

  // Visualization methods
  void createVisualization();
  void destroyVisualization();
  void updatePreviewArrow();

  // Interaction helpers
  bool getGroundPlaneIntersection(
    rviz_common::ViewportMouseEvent & event,
    Ogre::Vector3 & intersection);

  // Service communication
  void addWaypointToCurrentGroup();
  void insertWaypointAfterNearest();
  void deleteWaypointAtPosition(const Ogre::Vector3 & position);
  void requestGroupList();
  void updateGroupList();

  // Helper to find nearest waypoint
  bool findNearestWaypoint(
    const Ogre::Vector3 & position,
    int & group_index,
    int & waypoint_index,
    float max_distance = 0.5f);

  // Marker subscription callback
  void waypointMarkersCallback(
    const visualization_msgs::msg::MarkerArray::SharedPtr msg);

  // ROS2 communication
  rclcpp::Node::SharedPtr node_;

  // Service clients
  rclcpp::Client<srv::AddWaypoint>::SharedPtr add_waypoint_client_;
  rclcpp::Client<srv::InsertWaypoint>::SharedPtr insert_waypoint_client_;
  rclcpp::Client<srv::DeleteWaypoint>::SharedPtr delete_waypoint_client_;
  rclcpp::Client<srv::CreateGroup>::SharedPtr create_group_client_;
  rclcpp::Client<srv::GetGroupList>::SharedPtr get_group_list_client_;

  // Subscriber for waypoint visualization
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr
    waypoint_markers_sub_;

  // Interaction state
  EditMode current_mode_;
  Ogre::Vector3 cursor_position_;
  float preview_yaw_;
  int current_group_index_;

  // Waypoint data from markers (for deletion)
  struct WaypointMarkerData {
    int group_index;
    int waypoint_index;
    Ogre::Vector3 position;
  };
  std::vector<WaypointMarkerData> waypoint_positions_;

  // Visualization objects
  Ogre::SceneNode * scene_node_;
  std::unique_ptr<rviz_rendering::Arrow> preview_arrow_;

  // Properties (appear in RViz Tool Properties panel)
  rviz_common::properties::EnumProperty * edit_mode_property_;
  rviz_common::properties::EnumProperty * group_selection_property_;
  rviz_common::properties::FloatProperty * default_yaw_property_;
  rviz_common::properties::StringProperty * waypoint_file_property_;
  rviz_common::properties::ColorProperty * preview_color_property_;
  rviz_common::properties::FloatProperty * arrow_scale_property_;

  // State
  bool active_;
  bool preview_visible_;
  bool group_list_initialized_;
  size_t last_known_group_count_;
};

}  // namespace cub_mapping

#endif  // CUB_MAPPING__WAYPOINT_EDIT_TOOL_HPP_
