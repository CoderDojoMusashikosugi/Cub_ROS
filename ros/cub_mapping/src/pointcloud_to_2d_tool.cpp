// Copyright 2025 Cub Developer
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "cub_mapping/pointcloud_to_2d_tool.hpp"

#include <algorithm>
#include <memory>
#include <string>

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
#include <rviz_rendering/objects/shape.hpp>
#include <rviz_rendering/render_window.hpp>

#include <pluginlib/class_list_macros.hpp>

namespace cub_mapping
{

PointCloudTo2DTool::PointCloudTo2DTool()
: rviz_common::Tool(),
  node_(nullptr),
  cursor_position_(0.0f, 0.0f, 0.0f),
  height_z_(0.0f),
  region_width_(2.0f),
  height_range_(1.0f),
  scene_node_(nullptr),
  wireframe_box_(nullptr),
  cursor_line_(nullptr),
  active_(false)
{
  shortcut_key_ = 'z';  // Press 'z' to activate this tool
}

PointCloudTo2DTool::~PointCloudTo2DTool()
{
  destroyVisualization();
}

void PointCloudTo2DTool::onInitialize()
{
  // Create ROS2 node for communication
  node_ = context_->getRosNodeAbstraction().lock()->get_raw_node();

  // Create service client
  add_region_client_ = node_->create_client<cub_mapping::srv::AddRegion>(
    "add_region");

  // Create marker publisher for additional visualization if needed
  marker_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
    "mapping_tool_markers", 10);

  // Create publisher for slice height
  slice_height_pub_ = node_->create_publisher<std_msgs::msg::Float64>(
    "map_slice_height", 10);

  // Create properties
  height_property_ = new rviz_common::properties::FloatProperty(
    "Height (Z)", height_z_, "Height of the region center in Z direction",
    getPropertyContainer(), SLOT(updateProperties()), this);
  height_property_->setMin(-10.0f);
  height_property_->setMax(10.0f);

  width_property_ = new rviz_common::properties::FloatProperty(
    "Width", region_width_, "Width of the region in XY plane",
    getPropertyContainer(), SLOT(updateProperties()), this);
  width_property_->setMin(0.5f);
  width_property_->setMax(20.0f);

  range_property_ = new rviz_common::properties::FloatProperty(
    "Height Range", height_range_, "Height range in Z direction (±range/2)",
    getPropertyContainer(), SLOT(updateProperties()), this);
  range_property_->setMin(0.1f);
  range_property_->setMax(5.0f);

  color_property_ = new rviz_common::properties::ColorProperty(
    "Wireframe Color", QColor(0, 255, 0), "Color of the wireframe box",
    getPropertyContainer(), SLOT(updateProperties()), this);

  auto_send_property_ = new rviz_common::properties::BoolProperty(
    "Auto Send", false, "Automatically send region on mouse move",
    getPropertyContainer(), SLOT(updateProperties()), this);

  // Create scene node
  scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
}

void PointCloudTo2DTool::activate()
{
  active_ = true;
  createVisualization();
  publishSliceHeight();  // Send initial height to backend
  RCLCPP_INFO(node_->get_logger(), "PointCloud to 2D Tool activated");
}

void PointCloudTo2DTool::deactivate()
{
  active_ = false;
  destroyVisualization();
  RCLCPP_INFO(node_->get_logger(), "PointCloud to 2D Tool deactivated");
}

void PointCloudTo2DTool::createVisualization()
{
  if (!scene_node_) {
    return;
  }

  // Create wireframe box
  wireframe_box_ = std::make_unique<rviz_rendering::Shape>(
    rviz_rendering::Shape::Cube,
    scene_manager_,
    scene_node_);

  QColor color = color_property_->getColor();
  wireframe_box_->setColor(
    color.redF(), color.greenF(), color.blueF(), 0.3f);

  // Create cursor line
  cursor_line_ = std::make_unique<rviz_rendering::BillboardLine>(
    scene_manager_,
    scene_node_);
  cursor_line_->setColor(1.0f, 1.0f, 0.0f, 1.0f);  // Yellow
  cursor_line_->setLineWidth(0.02f);

  updateWireframe();
  updateCursorLine();
}

void PointCloudTo2DTool::destroyVisualization()
{
  wireframe_box_.reset();
  cursor_line_.reset();
}

void PointCloudTo2DTool::updateWireframe()
{
  if (!wireframe_box_ || !active_) {
    return;
  }

  // Update wireframe box position and size
  // Box bottom is at height_z_ (map_slice plane), so center is at height_z_ + height_range/2
  Ogre::Vector3 box_center(
    cursor_position_.x,
    cursor_position_.y,
    height_z_ + height_range_ / 2.0f);

  wireframe_box_->setPosition(box_center);
  wireframe_box_->setScale(Ogre::Vector3(
    region_width_,
    region_width_,
    height_range_));

  // Update color
  QColor color = color_property_->getColor();
  wireframe_box_->setColor(
    color.redF(), color.greenF(), color.blueF(), 0.3f);
}

void PointCloudTo2DTool::updateCursorLine()
{
  if (!cursor_line_ || !active_) {
    return;
  }

  // Draw vertical line from bottom to top
  cursor_line_->clear();
  cursor_line_->setNumLines(1);
  cursor_line_->setMaxPointsPerLine(2);

  Ogre::Vector3 line_start(cursor_position_.x, cursor_position_.y, -5.0f);
  Ogre::Vector3 line_end(cursor_position_.x, cursor_position_.y, 5.0f);

  cursor_line_->addPoint(line_start);
  cursor_line_->addPoint(line_end);
}

bool PointCloudTo2DTool::getGroundPlaneIntersection(
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

int PointCloudTo2DTool::processMouseEvent(rviz_common::ViewportMouseEvent & event)
{
  if (!active_) {
    return Render;
  }

  // Handle wheel events for parameter adjustment
  if (event.type == QEvent::Wheel) {
    float delta = event.wheel_delta / 120.0f * 0.1f;

    if (event.shift()) {
      // Shift + scroll: Adjust width
      region_width_ = std::clamp(region_width_ + delta, 0.5f, 20.0f);
      width_property_->setFloat(region_width_);
    } else if (event.control()) {
      // Ctrl + scroll: Adjust height range
      height_range_ = std::clamp(height_range_ + delta * 0.5f, 0.1f, 5.0f);
      range_property_->setFloat(height_range_);
    } else {
      // Normal scroll: Adjust height
      height_z_ += delta;
      height_property_->setFloat(height_z_);
      publishSliceHeight();  // Send height update to backend
    }

    updateWireframe();
    // Return Render only (not Finished) to keep the tool active
    return Render;
  }

  // Get intersection with ground plane
  Ogre::Vector3 intersection;
  if (getGroundPlaneIntersection(event, intersection)) {
    cursor_position_ = intersection;
    updateWireframe();
    updateCursorLine();

    // Left click to send region
    if (event.leftDown()) {
      sendRegionToBackend();
      return Render | Finished;
    }

    // Auto send if enabled
    if (auto_send_property_->getBool() && event.type == QEvent::MouseMove) {
      sendRegionToBackend();
    }
  }

  return Render;
}

int PointCloudTo2DTool::processKeyEvent(
  QKeyEvent * /*event*/,
  rviz_common::RenderPanel * /*panel*/)
{
  // Wheel events are handled in processMouseEvent
  return 0;
}

void PointCloudTo2DTool::updateProperties()
{
  // Update values from properties
  height_z_ = height_property_->getFloat();
  region_width_ = width_property_->getFloat();
  height_range_ = range_property_->getFloat();

  updateWireframe();
  context_->queueRender();
}

void PointCloudTo2DTool::sendRegionToBackend()
{
  if (!add_region_client_->service_is_ready()) {
    RCLCPP_WARN(
      node_->get_logger(),
      "Service 'add_region' is not available. Start the map converter node first.");
    return;
  }

  auto request = std::make_shared<cub_mapping::srv::AddRegion::Request>();
  request->center_x = cursor_position_.x;
  request->center_y = cursor_position_.y;
  request->center_z = height_z_ + height_range_ / 2.0;  // Center of the box
  request->width = region_width_;
  request->height_range = height_range_;

  RCLCPP_INFO(
    node_->get_logger(),
    "Sending region: center=(%.2f, %.2f, %.2f), width=%.2f, range=%.2f",
    request->center_x, request->center_y, request->center_z,
    request->width, request->height_range);

  // Async call
  auto result_future = add_region_client_->async_send_request(request);

  // Note: In a real implementation, you might want to handle the response
  // For now, we just send and forget
}

void PointCloudTo2DTool::publishSliceHeight()
{
  auto msg = std_msgs::msg::Float64();
  msg.data = height_z_;
  slice_height_pub_->publish(msg);

  RCLCPP_DEBUG(node_->get_logger(),
               "Published slice height: %.2f m", height_z_);
}

}  // namespace cub_mapping

PLUGINLIB_EXPORT_CLASS(cub_mapping::PointCloudTo2DTool, rviz_common::Tool)
