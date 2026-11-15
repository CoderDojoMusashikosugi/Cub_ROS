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
#include <rviz_common/properties/color_property.hpp>
#include <rviz_rendering/objects/arrow.hpp>
#include <std_msgs/msg/int32.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "cub_mapping/srv/add_waypoint.hpp"
#include "cub_mapping/srv/delete_waypoint.hpp"
#include "cub_mapping/srv/insert_waypoint.hpp"
#include "cub_mapping/srv/move_waypoint.hpp"

#include <OgreVector3.h>

namespace Ogre
{
class SceneNode;
}

namespace cub_mapping
{

/// Topic on which the Waypoint Editor panel publishes the target group index
constexpr char kCurrentGroupTopic[] = "waypoint_editor/current_group";

/**
 * @brief RViz2 tool for interactive waypoint editing
 *
 * This tool allows users to place and edit waypoints for navigation:
 * - Mouse movement: Position the waypoint cursor
 *   (existing waypoint under the cursor is highlighted)
 * - Mouse scroll: Adjust waypoint orientation (yaw)
 * - Left click on empty space: Place waypoint at end of current group
 * - Shift + Left click: Insert waypoint after nearest existing waypoint
 * - Left drag on a waypoint: Move it (right click while dragging cancels)
 * - Right click on a waypoint: Delete it
 * - Press 'w' to activate tool
 *
 * The target group for add/insert is selected in the Waypoint Editor panel
 * (received on kCurrentGroupTopic).
 *
 * The tool is modeless and uses no keyboard bindings, because letter keys
 * collide with other RViz tools' shortcuts and Esc always returns to the
 * default tool.
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

private:
  // Visualization methods
  void createVisualization();
  void destroyVisualization();
  void updatePreviewArrow();
  void setPreviewArrowPose(const Ogre::Vector3 & position, float yaw, bool highlight);
  void setPreviewVisible(bool visible);

  // Interaction helpers
  bool getGroundPlaneIntersection(
    rviz_common::ViewportMouseEvent & event,
    Ogre::Vector3 & intersection);

  // Service communication
  void addWaypointToCurrentGroup();
  void insertWaypointAfterNearest();
  void deleteWaypoint(int group_idx, int wp_idx);
  void updateHover();
  void startDrag(int group_idx, int wp_idx);
  void finishDrag(const Ogre::Vector3 & position);
  void cancelDrag();

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
  rclcpp::Client<srv::MoveWaypoint>::SharedPtr move_waypoint_client_;

  // Subscriber for waypoint visualization
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr
    waypoint_markers_sub_;
  // Subscriber for target group selected in the Waypoint Editor panel
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr current_group_sub_;

  // Interaction state
  Ogre::Vector3 cursor_position_;
  float preview_yaw_;
  int current_group_index_;

  // Waypoint data from markers (for deletion)
  struct WaypointMarkerData {
    int group_index;
    int waypoint_index;
    Ogre::Vector3 position;
    float yaw;
  };
  std::vector<WaypointMarkerData> waypoint_positions_;

  // Visualization objects
  Ogre::SceneNode * scene_node_;
  std::unique_ptr<rviz_rendering::Arrow> preview_arrow_;

  // Properties (appear in RViz Tool Properties panel)
  rviz_common::properties::FloatProperty * default_yaw_property_;
  rviz_common::properties::ColorProperty * preview_color_property_;
  rviz_common::properties::ColorProperty * highlight_color_property_;
  rviz_common::properties::FloatProperty * arrow_scale_property_;

  // State
  bool active_;
  bool preview_visible_;

  // Drag state (Move Position mode)
  bool dragging_;
  int drag_group_index_;
  int drag_waypoint_index_;
  float drag_yaw_;
  bool drag_moved_;
  Ogre::Vector3 drag_start_position_;

  // Waypoint currently under the cursor (-1 if none)
  int hover_group_index_;
  int hover_waypoint_index_;

  // Pick radius for hovering / dragging / deleting existing waypoints [m]
  static constexpr float kPickRadius = 0.5f;
  // Minimum cursor travel to treat a press as a drag [m]
  static constexpr float kDragThreshold = 0.05f;
};

}  // namespace cub_mapping

#endif  // CUB_MAPPING__WAYPOINT_EDIT_TOOL_HPP_
