// Copyright 2025 Cub Developer
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "cub_mapping/waypoint_edit_tool.hpp"

#include <cmath>
#include <set>

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <OgreEntity.h>
#include <OgreVector3.h>
#include <OgrePlane.h>
#include <OgreRay.h>
#include <OgreCamera.h>
#include <OgreViewport.h>

#include <QKeyEvent>
#include <QTimer>

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
  current_mode_(MODE_ADD_WAYPOINT),
  cursor_position_(0.0f, 0.0f, 0.0f),
  preview_yaw_(0.0f),
  current_group_index_(0),
  scene_node_(nullptr),
  active_(false),
  preview_visible_(false),
  group_list_initialized_(false),
  last_known_group_count_(0)
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
  create_group_client_ = node_->create_client<srv::CreateGroup>("create_group");
  get_group_list_client_ = node_->create_client<srv::GetGroupList>("get_group_list");

  // Subscribe to waypoint markers
  waypoint_markers_sub_ = node_->create_subscription<visualization_msgs::msg::MarkerArray>(
    "waypoint_markers", 10,
    std::bind(&WaypointEditTool::waypointMarkersCallback, this, std::placeholders::_1));

  // Create properties
  edit_mode_property_ = new rviz_common::properties::EnumProperty(
    "Edit Mode", "Add Waypoint",
    "Current editing mode",
    getPropertyContainer(), SLOT(updateProperties()), this);
  edit_mode_property_->addOption("Add Waypoint", MODE_ADD_WAYPOINT);
  edit_mode_property_->addOption("Delete Waypoint", MODE_DELETE_WAYPOINT);
  edit_mode_property_->addOption("Select", MODE_SELECT);
  edit_mode_property_->addOption("Move Position", MODE_MOVE_POSITION);
  edit_mode_property_->addOption("Rotate Orientation", MODE_ROTATE_ORIENTATION);

  group_selection_property_ = new rviz_common::properties::EnumProperty(
    "Current Group", "Group 0",
    "Select current waypoint group",
    getPropertyContainer(), SLOT(updateCurrentGroup()), this);
  // Will be populated when we receive marker data
  group_selection_property_->addOption("Group 0", 0);

  default_yaw_property_ = new rviz_common::properties::FloatProperty(
    "Default Yaw", 0.0f, "Default yaw angle for new waypoints (radians)",
    getPropertyContainer(), SLOT(updateProperties()), this);
  default_yaw_property_->setMin(-3.14159f);
  default_yaw_property_->setMax(3.14159f);

  waypoint_file_property_ = new rviz_common::properties::StringProperty(
    "Waypoint File", "",
    "Path to waypoint YAML file",
    getPropertyContainer(), SLOT(updateProperties()), this);

  preview_color_property_ = new rviz_common::properties::ColorProperty(
    "Preview Color", QColor(0, 255, 255),
    "Color of the preview arrow",
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

  // Request group list to populate dropdown (only once)
  if (!group_list_initialized_) {
    updateGroupList();
    group_list_initialized_ = true;
  }

  RCLCPP_INFO(node_->get_logger(), "Waypoint Edit Tool activated");
}

void WaypointEditTool::deactivate()
{
  active_ = false;
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

void WaypointEditTool::updatePreviewArrow()
{
  if (!preview_arrow_ || !preview_visible_) {
    return;
  }

  // Set position
  preview_arrow_->setPosition(Ogre::Vector3(
    cursor_position_.x,
    cursor_position_.y,
    cursor_position_.z + 0.01f));  // Slightly above ground

  // Set orientation based on yaw
  // Ogre::Arrow points along +Z by default, so we need to:
  // 1. Rotate -90° around Y to make it point along +X (forward)
  // 2. Rotate by yaw around Z axis
  Ogre::Quaternion pitch_rotation;
  pitch_rotation.FromAngleAxis(Ogre::Radian(-Ogre::Math::HALF_PI), Ogre::Vector3::UNIT_Y);

  Ogre::Quaternion yaw_rotation;
  yaw_rotation.FromAngleAxis(Ogre::Radian(preview_yaw_), Ogre::Vector3::UNIT_Z);

  // Combine rotations: first pitch down to XY plane, then yaw
  Ogre::Quaternion orientation = yaw_rotation * pitch_rotation;
  preview_arrow_->setOrientation(orientation);

  // Update color
  QColor color = preview_color_property_->getColor();
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

  // Handle scroll wheel for yaw adjustment (process first to prevent propagation)
  if (event.type == QEvent::Wheel) {
    if (current_mode_ == MODE_ADD_WAYPOINT) {
      float delta = event.wheel_delta / 120.0f;  // Normalized wheel delta

      // Adjust yaw based on scroll direction
      float yaw_step = 0.1f;  // radians per scroll step
      preview_yaw_ += delta * yaw_step;

      // Keep yaw in [-pi, pi] range
      while (preview_yaw_ > M_PI) preview_yaw_ -= 2.0f * M_PI;
      while (preview_yaw_ < -M_PI) preview_yaw_ += 2.0f * M_PI;

      updatePreviewArrow();
    }
    // IMPORTANT: Return Render only (not Finished) to keep tool active
    // Finished flag would cause the tool to switch back to default
    return Render;
  }

  Ogre::Vector3 intersection;
  bool intersects = getGroundPlaneIntersection(event, intersection);

  // Update cursor position
  if (intersects) {
    cursor_position_ = intersection;
  }

  // Handle different modes
  if (current_mode_ == MODE_ADD_WAYPOINT) {
    // Show preview arrow
    if (intersects) {
      if (!preview_visible_) {
        preview_visible_ = true;
        if (preview_arrow_) {
          preview_arrow_->getSceneNode()->setVisible(true);
        }
      }
      updatePreviewArrow();
    }

    // Handle left click to place waypoint
    if (event.leftDown() && intersects) {
      // Shift + Click: Insert waypoint after nearest existing waypoint
      if (event.shift()) {
        insertWaypointAfterNearest();
      } else {
        // Normal Click: Add waypoint to end of group
        addWaypointToCurrentGroup();
      }
      return Render;
    }
  } else if (current_mode_ == MODE_DELETE_WAYPOINT) {
    // Hide preview arrow in delete mode
    if (preview_visible_) {
      preview_visible_ = false;
      if (preview_arrow_) {
        preview_arrow_->getSceneNode()->setVisible(false);
      }
    }

    // Handle left click to delete waypoint
    if (event.leftDown() && intersects) {
      deleteWaypointAtPosition(cursor_position_);
      return Render;
    }
  }

  // Return Render to keep the tool active
  return Render;
}

int WaypointEditTool::processKeyEvent(
  QKeyEvent * event,
  rviz_common::RenderPanel * /*panel*/)
{
  if (!active_) {
    return 0;
  }

  // Handle mode switching keys
  if (event->key() == Qt::Key_A) {
    current_mode_ = MODE_ADD_WAYPOINT;
    edit_mode_property_->setValue(MODE_ADD_WAYPOINT);
    RCLCPP_INFO(node_->get_logger(), "Switched to Add Waypoint mode");
    return Render;
  } else if (event->key() == Qt::Key_D) {
    current_mode_ = MODE_DELETE_WAYPOINT;
    edit_mode_property_->setValue(MODE_DELETE_WAYPOINT);
    RCLCPP_INFO(node_->get_logger(), "Switched to Delete Waypoint mode");
    return Render;
  } else if (event->key() == Qt::Key_S) {
    current_mode_ = MODE_SELECT;
    edit_mode_property_->setValue(MODE_SELECT);
    RCLCPP_INFO(node_->get_logger(), "Switched to Select mode");
    return Render;
  } else if (event->key() == Qt::Key_M) {
    current_mode_ = MODE_MOVE_POSITION;
    edit_mode_property_->setValue(MODE_MOVE_POSITION);
    RCLCPP_INFO(node_->get_logger(), "Switched to Move Position mode");
    return Render;
  } else if (event->key() == Qt::Key_R) {
    current_mode_ = MODE_ROTATE_ORIENTATION;
    edit_mode_property_->setValue(MODE_ROTATE_ORIENTATION);
    RCLCPP_INFO(node_->get_logger(), "Switched to Rotate Orientation mode");
    return Render;
  }

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

  // Track unique group indices to detect group count changes
  std::set<int> unique_groups;

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

          unique_groups.insert(group_idx);

          WaypointMarkerData data;
          data.group_index = group_idx;
          data.waypoint_index = wp_idx;
          data.position = Ogre::Vector3(
            marker.pose.position.x,
            marker.pose.position.y,
            marker.pose.position.z);

          waypoint_positions_.push_back(data);
        } catch (const std::exception&) {
          // Skip malformed marker names
        }
      }
    }
  }

  // Check if the number of groups has changed
  if (unique_groups.size() != last_known_group_count_) {
    last_known_group_count_ = unique_groups.size();
    RCLCPP_INFO(node_->get_logger(), "Group count changed to %zu, updating group list",
                last_known_group_count_);
    updateGroupList();
  }
}

void WaypointEditTool::updateProperties()
{
  if (!active_) {
    return;
  }

  // Update current mode from property
  current_mode_ = static_cast<EditMode>(edit_mode_property_->getOptionInt());

  // Update default yaw
  preview_yaw_ = default_yaw_property_->getFloat();

  // Update visualization
  updatePreviewArrow();
}

void WaypointEditTool::updateCurrentGroup()
{
  current_group_index_ = group_selection_property_->getOptionInt();
  RCLCPP_INFO(node_->get_logger(), "Switched to group index: %d", current_group_index_);
}

void WaypointEditTool::updateGroupList()
{
  // Wait for service to become available (with timeout)
  if (!get_group_list_client_->wait_for_service(std::chrono::milliseconds(100))) {
    RCLCPP_WARN(node_->get_logger(), "get_group_list service not available yet, will retry");

    // Retry after a delay using QTimer
    QTimer::singleShot(500, [this]() {
      if (!group_list_initialized_) {
        RCLCPP_INFO(node_->get_logger(), "Retrying group list update...");
        updateGroupList();
      }
    });
    return;
  }

  auto request = std::make_shared<srv::GetGroupList::Request>();

  get_group_list_client_->async_send_request(
    request,
    [this](rclcpp::Client<srv::GetGroupList>::SharedFuture future) {
      try {
        auto result = future.get();

        // Save current selection index before clearing
        int previous_selection = current_group_index_;

        // Clear existing options
        group_selection_property_->clearOptions();

        // Add each group with its name
        for (size_t i = 0; i < result->group_names.size(); ++i) {
          std::string label = result->group_names[i];

          // Add require_input indicator if needed
          if (i < result->require_input_flags.size() && result->require_input_flags[i]) {
            label += " *";
          }

          group_selection_property_->addOptionStd(label, static_cast<int>(i));
        }

        // If no groups exist, add a default one
        if (result->group_names.empty()) {
          group_selection_property_->addOption("Group 0", 0);
          RCLCPP_WARN(node_->get_logger(), "No groups found, showing default");
          current_group_index_ = 0;
          last_known_group_count_ = 0;
        } else {
          RCLCPP_INFO(node_->get_logger(), "Loaded %zu groups:", result->group_names.size());
          for (size_t i = 0; i < result->group_names.size(); ++i) {
            RCLCPP_INFO(node_->get_logger(), "  [%zu] %s", i, result->group_names[i].c_str());
          }

          // Update the known group count
          last_known_group_count_ = result->group_names.size();

          // Restore previous selection if still valid
          if (previous_selection >= 0 &&
              previous_selection < static_cast<int>(result->group_names.size())) {
            current_group_index_ = previous_selection;
          } else {
            current_group_index_ = 0;
          }
        }

        // Set the property to the current selection
        group_selection_property_->setValue(current_group_index_);

      } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to get group list: %s", e.what());
      }
    });
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

void WaypointEditTool::deleteWaypointAtPosition(const Ogre::Vector3 & position)
{
  int group_idx = -1;
  int wp_idx = -1;

  if (!findNearestWaypoint(position, group_idx, wp_idx, 0.5f)) {
    RCLCPP_WARN(node_->get_logger(), "No waypoint found near click position");
    return;
  }

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

}  // namespace cub_mapping

PLUGINLIB_EXPORT_CLASS(cub_mapping::WaypointEditTool, rviz_common::Tool)
