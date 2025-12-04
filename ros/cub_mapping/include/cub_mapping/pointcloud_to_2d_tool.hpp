// Copyright 2025 Cub Developer
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#ifndef CUB_MAPPING__POINTCLOUD_TO_2D_TOOL_HPP_
#define CUB_MAPPING__POINTCLOUD_TO_2D_TOOL_HPP_

#include <memory>
#include <string>

#include <QObject>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/tool.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/viewport_mouse_event.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_rendering/objects/shape.hpp>
#include <rviz_rendering/objects/billboard_line.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/float64.hpp>

#include "cub_mapping/srv/add_region.hpp"

#include <OgreVector3.h>

namespace Ogre
{
class SceneNode;
}

namespace cub_mapping
{

/**
 * @brief RViz2 tool for interactive 3D point cloud to 2D map conversion
 *
 * This tool allows users to select regions in 3D space using mouse and keyboard:
 * - Mouse movement: Position the region in XY plane
 * - Mouse scroll: Adjust height (Z position)
 * - Shift + scroll: Adjust region width
 * - Ctrl + scroll: Adjust height range
 * - Left click: Confirm and send region to backend
 */
class PointCloudTo2DTool : public rviz_common::Tool
{
  Q_OBJECT

public:
  PointCloudTo2DTool();
  virtual ~PointCloudTo2DTool();

  void onInitialize() override;
  void activate() override;
  void deactivate() override;

  int processMouseEvent(rviz_common::ViewportMouseEvent & event) override;
  int processKeyEvent(QKeyEvent * event, rviz_common::RenderPanel * panel) override;

private Q_SLOTS:
  void updateProperties();

private:
  void createVisualization();
  void destroyVisualization();
  void updateWireframe();
  void updateCursorLine();
  void sendRegionToBackend();
  void publishSliceHeight();

  bool getGroundPlaneIntersection(
    rviz_common::ViewportMouseEvent & event,
    Ogre::Vector3 & intersection);

  // ROS2 communication
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<cub_mapping::srv::AddRegion>::SharedPtr add_region_client_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr slice_height_pub_;

  // Region parameters
  Ogre::Vector3 cursor_position_;
  float height_z_;          // Height in Z direction (m)
  float region_width_;      // Width of region in XY plane (m)
  float height_range_;      // Height range in Z direction (±height_range/2) (m)

  // Visualization objects
  Ogre::SceneNode * scene_node_;
  std::unique_ptr<rviz_rendering::Shape> wireframe_box_;
  std::unique_ptr<rviz_rendering::BillboardLine> cursor_line_;

  // Properties displayed in RViz Tool Properties panel
  rviz_common::properties::FloatProperty * height_property_;
  rviz_common::properties::FloatProperty * width_property_;
  rviz_common::properties::FloatProperty * range_property_;
  rviz_common::properties::ColorProperty * color_property_;
  rviz_common::properties::BoolProperty * auto_send_property_;

  // State
  bool active_;
};

}  // namespace cub_mapping

#endif  // CUB_MAPPING__POINTCLOUD_TO_2D_TOOL_HPP_
