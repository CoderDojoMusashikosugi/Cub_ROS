// Copyright 2025 Cub Developer
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "cub_mapping/waypoint_edit_tool.hpp"

#include <cmath>

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <OgreEntity.h>
#include <OgreVector3.h>
#include <OgrePlane.h>
#include <OgreRay.h>
#include <OgreCamera.h>
#include <OgreViewport.h>

#include <QKeyEvent>

#include <rviz_common/display_context.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/viewport_mouse_event.hpp>
#include <rviz_rendering/geometry.hpp>
#include <rviz_rendering/render_window.hpp>

#include <pluginlib/class_list_macros.hpp>

namespace cub_mapping
{

WaypointEditTool::WaypointEditTool()
: rviz_common::Tool(),
  node_(nullptr),
  cursor_position_(0.0f, 0.0f, 0.0f),
  preview_yaw_(0.0f),
  current_group_index_(0),
  scene_node_(nullptr),
  active_(false),
  preview_visible_(false),
  dragging_(false),
  drag_group_index_(-1),
  drag_waypoint_index_(-1),
  drag_yaw_(0.0f),
  drag_moved_(false),
  hover_group_index_(-1),
  hover_waypoint_index_(-1)
{
  shortcut_key_ = 'w';  // Press 'w' to activate this tool
}

WaypointEditTool::~WaypointEditTool()
{
  destroyVisualization();
}

void WaypointEditTool::onInitialize()
{
  // Create ROS2 node for communication
  node_ = context_->getRosNodeAbstraction().lock()->get_raw_node();

  // Create service clients
  add_waypoint_client_ = node_->create_client<srv::AddWaypoint>("add_waypoint");
  insert_waypoint_client_ = node_->create_client<srv::InsertWaypoint>("insert_waypoint");
  delete_waypoint_client_ = node_->create_client<srv::DeleteWaypoint>("delete_waypoint");
  move_waypoint_client_ = node_->create_client<srv::MoveWaypoint>("move_waypoint");

  // Subscribe to waypoint markers
  waypoint_markers_sub_ = node_->create_subscription<visualization_msgs::msg::MarkerArray>(
    "waypoint_markers", 10,
    std::bind(&WaypointEditTool::waypointMarkersCallback, this, std::placeholders::_1));

  // Target group is selected in the Waypoint Editor panel
  current_group_sub_ = node_->create_subscription<std_msgs::msg::Int32>(
    kCurrentGroupTopic, rclcpp::QoS(1).transient_local(),
    [this](const std_msgs::msg::Int32::SharedPtr msg) {
      current_group_index_ = msg->data;
      RCLCPP_INFO(node_->get_logger(), "Target group index: %d", current_group_index_);
    });

  // Create properties
  default_yaw_property_ = new rviz_common::properties::FloatProperty(
    "Default Yaw", 0.0f, "Default yaw angle for new waypoints (radians)",
    getPropertyContainer(), SLOT(updateProperties()), this);
  default_yaw_property_->setMin(-3.14159f);
  default_yaw_property_->setMax(3.14159f);

  preview_color_property_ = new rviz_common::properties::ColorProperty(
    "Preview Color", QColor(0, 255, 255),
    "Color of the preview arrow",
    getPropertyContainer(), SLOT(updateProperties()), this);

  highlight_color_property_ = new rviz_common::properties::ColorProperty(
    "Highlight Color", QColor(255, 140, 0),
    "Color of the arrow when hovering or dragging an existing waypoint",
    getPropertyContainer(), SLOT(updateProperties()), this);

  arrow_scale_property_ = new rviz_common::properties::FloatProperty(
    "Arrow Scale", 0.5f, "Scale of waypoint arrows",
    getPropertyContainer(), SLOT(updateProperties()), this);
  arrow_scale_property_->setMin(0.1f);
  arrow_scale_property_->setMax(2.0f);

  // Create scene node
  scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();

  RCLCPP_INFO(node_->get_logger(), "Waypoint Edit Tool initialized");
}

void WaypointEditTool::activate()
{
  active_ = true;
  createVisualization();
  preview_yaw_ = default_yaw_property_->getFloat();

  RCLCPP_INFO(node_->get_logger(), "Waypoint Edit Tool activated");
}

void WaypointEditTool::deactivate()
{
  active_ = false;
  dragging_ = false;
  destroyVisualization();
  RCLCPP_INFO(node_->get_logger(), "Waypoint Edit Tool deactivated");
}

void WaypointEditTool::createVisualization()
{
  if (!scene_node_) {
    return;
  }

  // Create preview arrow
  preview_arrow_ = std::make_unique<rviz_rendering::Arrow>(
    scene_manager_,
    scene_node_);

  QColor color = preview_color_property_->getColor();
  preview_arrow_->setColor(
    color.redF(), color.greenF(), color.blueF(), 0.7f);

  float scale = arrow_scale_property_->getFloat();
  preview_arrow_->set(scale, scale * 0.2f, scale * 0.3f, scale * 0.2f);

  preview_visible_ = false;
  preview_arrow_->getSceneNode()->setVisible(false);
}

void WaypointEditTool::destroyVisualization()
{
  preview_arrow_.reset();
  preview_visible_ = false;
}

void WaypointEditTool::setPreviewVisible(bool visible)
{
  if (preview_visible_ == visible) {
    return;
  }
  preview_visible_ = visible;
  if (preview_arrow_) {
    preview_arrow_->getSceneNode()->setVisible(visible);
  }
}

void WaypointEditTool::updatePreviewArrow()
{
  if (hover_group_index_ >= 0) {
    // Highlight the waypoint under the cursor (can be dragged / deleted)
    for (const auto & wp_data : waypoint_positions_) {
      if (wp_data.group_index == hover_group_index_ &&
          wp_data.waypoint_index == hover_waypoint_index_)
      {
        setPreviewArrowPose(wp_data.position, wp_data.yaw, true);
        return;
      }
    }
  }
  setPreviewArrowPose(cursor_position_, preview_yaw_, false);
}

void WaypointEditTool::setPreviewArrowPose(
  const Ogre::Vector3 & position, float yaw,
  bool highlight)
{
  if (!preview_arrow_ || !preview_visible_) {
    return;
  }

  // Set position
  preview_arrow_->setPosition(Ogre::Vector3(
    position.x,
    position.y,
    position.z + 0.01f));  // Slightly above ground

  // Set orientation based on yaw
  // Ogre::Arrow points along +Z by default, so we need to:
  // 1. Rotate -90° around Y to make it point along +X (forward)
  // 2. Rotate by yaw around Z axis
  Ogre::Quaternion pitch_rotation;
  pitch_rotation.FromAngleAxis(Ogre::Radian(-Ogre::Math::HALF_PI), Ogre::Vector3::UNIT_Y);

  Ogre::Quaternion yaw_rotation;
  yaw_rotation.FromAngleAxis(Ogre::Radian(yaw), Ogre::Vector3::UNIT_Z);

  // Combine rotations: first pitch down to XY plane, then yaw
  Ogre::Quaternion orientation = yaw_rotation * pitch_rotation;
  preview_arrow_->setOrientation(orientation);

  // Update color
  QColor color = highlight ?
    highlight_color_property_->getColor() : preview_color_property_->getColor();
  preview_arrow_->setColor(
    color.redF(), color.greenF(), color.blueF(), 0.7f);

  // Update scale
  float scale = arrow_scale_property_->getFloat();
  preview_arrow_->set(scale, scale * 0.2f, scale * 0.3f, scale * 0.2f);
}

bool WaypointEditTool::getGroundPlaneIntersection(
  rviz_common::ViewportMouseEvent & event,
  Ogre::Vector3 & intersection)
{
  // Get the camera from the render panel
  auto render_window = event.panel->getRenderWindow();
  if (!render_window) {
    return false;
  }

  Ogre::Camera * camera = rviz_rendering::RenderWindowOgreAdapter::getOgreCamera(render_window);
  if (!camera) {
    return false;
  }

  Ogre::Viewport * viewport = rviz_rendering::RenderWindowOgreAdapter::getOgreViewport(render_window);
  if (!viewport) {
    return false;
  }

  int width = viewport->getActualWidth();
  int height = viewport->getActualHeight();

  // Calculate normalized screen coordinates [0,1]
  float screen_x = static_cast<float>(event.x) / static_cast<float>(width);
  float screen_y = static_cast<float>(event.y) / static_cast<float>(height);

  // Get ray from camera through screen point
  Ogre::Ray ray = camera->getCameraToViewportRay(screen_x, screen_y);

  // Intersect with XY plane at Z=0
  Ogre::Plane ground_plane(Ogre::Vector3::UNIT_Z, 0.0f);

  std::pair<bool, Ogre::Real> result = ray.intersects(ground_plane);
  if (result.first) {
    intersection = ray.getPoint(result.second);
    return true;
  }

  return false;
}

int WaypointEditTool::processMouseEvent(rviz_common::ViewportMouseEvent & event)
{
  if (!active_) {
    return 0;
  }

  // Scroll wheel: adjust yaw of the next waypoint to be added
  if (event.type == QEvent::Wheel) {
    if (!dragging_) {
      float delta = event.wheel_delta / 120.0f;  // Normalized wheel delta
      float yaw_step = 0.1f;  // radians per scroll step
      preview_yaw_ += delta * yaw_step;

      // Keep yaw in [-pi, pi] range
      while (preview_yaw_ > M_PI) preview_yaw_ -= 2.0f * M_PI;
      while (preview_yaw_ < -M_PI) preview_yaw_ += 2.0f * M_PI;

      updatePreviewArrow();
    }
    // IMPORTANT: Return Render only (not Finished) to keep tool active
    return Render;
  }

  Ogre::Vector3 intersection;
  bool intersects = getGroundPlaneIntersection(event, intersection);
  if (intersects) {
    cursor_position_ = intersection;
  }

  // ---- Dragging an existing waypoint ----
  if (dragging_) {
    if (event.rightDown()) {
      RCLCPP_INFO(node_->get_logger(), "Move cancelled");
      cancelDrag();
      updatePreviewArrow();
      return Render;
    }

    if (intersects) {
      Ogre::Vector3 diff = cursor_position_ - drag_start_position_;
      if (diff.x * diff.x + diff.y * diff.y > kDragThreshold * kDragThreshold) {
        drag_moved_ = true;
      }
      setPreviewArrowPose(cursor_position_, drag_yaw_, true);
    }

    if (event.leftUp()) {
      if (!drag_moved_) {
        // Simple click on an existing waypoint: do nothing
        cancelDrag();
      } else if (intersects) {
        finishDrag(cursor_position_);
      } else {
        RCLCPP_WARN(node_->get_logger(), "Released outside ground plane, move cancelled");
        cancelDrag();
      }
      updatePreviewArrow();
    } else if (!event.left()) {
      // Button released without receiving leftUp (e.g. outside the viewport)
      cancelDrag();
      updatePreviewArrow();
    }
    return Render;
  }

  if (!intersects) {
    return Render;
  }

  // ---- Hover feedback ----
  updateHover();
  setPreviewVisible(true);
  updatePreviewArrow();

  // ---- Clicks ----
  if (event.leftDown()) {
    if (event.shift()) {
      // Shift + Click: insert after nearest waypoint
      insertWaypointAfterNearest();
    } else if (hover_group_index_ >= 0) {
      // Click on existing waypoint: start dragging it
      startDrag(hover_group_index_, hover_waypoint_index_);
    } else {
      // Click on empty space: append to current group
      addWaypointToCurrentGroup();
    }
  } else if (event.rightDown() && hover_group_index_ >= 0) {
    // Right click on existing waypoint: delete it
    deleteWaypoint(hover_group_index_, hover_waypoint_index_);
    hover_group_index_ = -1;
    hover_waypoint_index_ = -1;
  }

  return Render;
}

int WaypointEditTool::processKeyEvent(
  QKeyEvent * /*event*/,
  rviz_common::RenderPanel * /*panel*/)
{
  // No key bindings: letter keys collide with other RViz tools' shortcuts
  // (e.g. 'm' = Move Camera, 's' = Select) and Esc always switches to the
  // default tool, so all editing is done with the mouse.
  return 0;
}

void WaypointEditTool::addWaypointToCurrentGroup()
{
  if (!add_waypoint_client_->service_is_ready()) {
    RCLCPP_WARN(node_->get_logger(), "add_waypoint service not available");
    return;
  }

  auto request = std::make_shared<srv::AddWaypoint::Request>();
  request->group_index = current_group_index_;
  request->x = cursor_position_.x;
  request->y = cursor_position_.y;
  request->yaw = preview_yaw_;

  // Send async request with callback (non-blocking)
  add_waypoint_client_->async_send_request(
    request,
    [this, request](rclcpp::Client<srv::AddWaypoint>::SharedFuture future) {
      try {
        auto result = future.get();
        if (result->success) {
          RCLCPP_INFO(node_->get_logger(), "Added waypoint: (%.2f, %.2f, %.2f) - %s",
                      request->x, request->y, request->yaw, result->message.c_str());
        } else {
          RCLCPP_ERROR(node_->get_logger(), "Failed to add waypoint: %s", result->message.c_str());
        }
      } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Service call exception: %s", e.what());
      }
    });
}

void WaypointEditTool::insertWaypointAfterNearest()
{
  // Find the nearest waypoint to the cursor position
  int nearest_group_idx = -1;
  int nearest_wp_idx = -1;

  if (!findNearestWaypoint(cursor_position_, nearest_group_idx, nearest_wp_idx, 5.0f)) {
    RCLCPP_WARN(node_->get_logger(), "No waypoint found near cursor position. Adding to end instead.");
    addWaypointToCurrentGroup();
    return;
  }

  // Check if the nearest waypoint is in the current group
  if (nearest_group_idx != current_group_index_) {
    RCLCPP_WARN(node_->get_logger(),
                "Nearest waypoint is in group %d, but current group is %d. Adding to end of current group instead.",
                nearest_group_idx, current_group_index_);
    addWaypointToCurrentGroup();
    return;
  }

  if (!insert_waypoint_client_->service_is_ready()) {
    RCLCPP_WARN(node_->get_logger(), "insert_waypoint service not available");
    return;
  }

  // Insert after the nearest waypoint (insert_before_index = nearest_wp_idx + 1)
  auto request = std::make_shared<srv::InsertWaypoint::Request>();
  request->group_index = current_group_index_;
  request->insert_before_index = nearest_wp_idx + 1;
  request->x = cursor_position_.x;
  request->y = cursor_position_.y;
  request->yaw = preview_yaw_;

  // Send async request with callback (non-blocking)
  insert_waypoint_client_->async_send_request(
    request,
    [this, request, nearest_wp_idx](rclcpp::Client<srv::InsertWaypoint>::SharedFuture future) {
      try {
        auto result = future.get();
        if (result->success) {
          RCLCPP_INFO(node_->get_logger(),
                      "Inserted waypoint after index %d: (%.2f, %.2f, %.2f) - %s",
                      nearest_wp_idx, request->x, request->y, request->yaw, result->message.c_str());
        } else {
          RCLCPP_ERROR(node_->get_logger(), "Failed to insert waypoint: %s", result->message.c_str());
        }
      } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Service call exception: %s", e.what());
      }
    });
}

void WaypointEditTool::waypointMarkersCallback(
  const visualization_msgs::msg::MarkerArray::SharedPtr msg)
{
  // Clear previous waypoint positions
  waypoint_positions_.clear();

  // Extract waypoint positions from arrow markers
  for (const auto & marker : msg->markers) {
    // Look for arrow markers (type 0)
    if (marker.type == visualization_msgs::msg::Marker::ARROW &&
        marker.ns.find("waypoint_") == 0) {
      // Parse namespace: "waypoint_<group>_<index>"
      std::string ns = marker.ns;
      size_t first_underscore = ns.find('_');
      size_t second_underscore = ns.find('_', first_underscore + 1);

      if (second_underscore != std::string::npos) {
        try {
          int group_idx = std::stoi(ns.substr(first_underscore + 1,
                                               second_underscore - first_underscore - 1));
          int wp_idx = std::stoi(ns.substr(second_underscore + 1));

          WaypointMarkerData data;
          data.group_index = group_idx;
          data.waypoint_index = wp_idx;
          data.position = Ogre::Vector3(
            marker.pose.position.x,
            marker.pose.position.y,
            marker.pose.position.z);
          const auto & q = marker.pose.orientation;
          data.yaw = static_cast<float>(std::atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)));

          waypoint_positions_.push_back(data);
        } catch (const std::exception&) {
          // Skip malformed marker names
        }
      }
    }
  }
}

void WaypointEditTool::updateProperties()
{
  if (!active_) {
    return;
  }

  // Update default yaw
  preview_yaw_ = default_yaw_property_->getFloat();

  // Update visualization
  updatePreviewArrow();
}

bool WaypointEditTool::findNearestWaypoint(
  const Ogre::Vector3 & position,
  int & group_index,
  int & waypoint_index,
  float max_distance)
{
  float min_dist_sq = max_distance * max_distance;
  bool found = false;

  for (const auto & wp_data : waypoint_positions_) {
    Ogre::Vector3 diff = wp_data.position - position;
    // Only check XY distance (ignore Z)
    float dist_sq = diff.x * diff.x + diff.y * diff.y;

    if (dist_sq < min_dist_sq) {
      min_dist_sq = dist_sq;
      group_index = wp_data.group_index;
      waypoint_index = wp_data.waypoint_index;
      found = true;
    }
  }

  return found;
}

void WaypointEditTool::deleteWaypoint(int group_idx, int wp_idx)
{
  if (!delete_waypoint_client_->service_is_ready()) {
    RCLCPP_WARN(node_->get_logger(), "delete_waypoint service not available");
    return;
  }

  auto request = std::make_shared<srv::DeleteWaypoint::Request>();
  request->group_index = group_idx;
  request->waypoint_index = wp_idx;

  // Send async request with callback (non-blocking)
  delete_waypoint_client_->async_send_request(
    request,
    [this, group_idx, wp_idx](rclcpp::Client<srv::DeleteWaypoint>::SharedFuture future) {
      try {
        auto result = future.get();
        if (result->success) {
          RCLCPP_INFO(node_->get_logger(), "Deleted waypoint [group %d, index %d] - %s",
                      group_idx, wp_idx, result->message.c_str());
        } else {
          RCLCPP_ERROR(node_->get_logger(), "Failed to delete waypoint: %s",
                       result->message.c_str());
        }
      } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Service call exception: %s", e.what());
      }
    });
}

void WaypointEditTool::startDrag(int group_idx, int wp_idx)
{
  drag_yaw_ = 0.0f;
  for (const auto & wp_data : waypoint_positions_) {
    if (wp_data.group_index == group_idx && wp_data.waypoint_index == wp_idx) {
      drag_yaw_ = wp_data.yaw;
      break;
    }
  }

  dragging_ = true;
  drag_moved_ = false;
  drag_group_index_ = group_idx;
  drag_waypoint_index_ = wp_idx;
  drag_start_position_ = cursor_position_;

  setPreviewVisible(true);
  setPreviewArrowPose(cursor_position_, drag_yaw_, true);
}

void WaypointEditTool::updateHover()
{
  hover_group_index_ = -1;
  hover_waypoint_index_ = -1;
  findNearestWaypoint(cursor_position_, hover_group_index_, hover_waypoint_index_,
                      kPickRadius);
}

void WaypointEditTool::cancelDrag()
{
  dragging_ = false;
  drag_moved_ = false;
  drag_group_index_ = -1;
  drag_waypoint_index_ = -1;
}

void WaypointEditTool::finishDrag(const Ogre::Vector3 & position)
{
  const int group_idx = drag_group_index_;
  const int wp_idx = drag_waypoint_index_;
  cancelDrag();

  if (!move_waypoint_client_->service_is_ready()) {
    RCLCPP_WARN(node_->get_logger(), "move_waypoint service not available");
    return;
  }

  auto request = std::make_shared<srv::MoveWaypoint::Request>();
  request->group_index = group_idx;
  request->waypoint_index = wp_idx;
  request->new_x = position.x;
  request->new_y = position.y;

  // Send async request with callback (non-blocking)
  move_waypoint_client_->async_send_request(
    request,
    [this, request](rclcpp::Client<srv::MoveWaypoint>::SharedFuture future) {
      try {
        auto result = future.get();
        if (result->success) {
          RCLCPP_INFO(node_->get_logger(),
                      "Moved waypoint [group %d, index %d] to (%.2f, %.2f) - %s",
                      request->group_index, request->waypoint_index,
                      request->new_x, request->new_y, result->message.c_str());
        } else {
          RCLCPP_ERROR(node_->get_logger(), "Failed to move waypoint: %s",
                       result->message.c_str());
        }
      } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Service call exception: %s", e.what());
      }
    });
}

}  // namespace cub_mapping

PLUGINLIB_EXPORT_CLASS(cub_mapping::WaypointEditTool, rviz_common::Tool)
