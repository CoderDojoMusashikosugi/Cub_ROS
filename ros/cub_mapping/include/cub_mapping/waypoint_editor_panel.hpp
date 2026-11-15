// Copyright 2025 Cub Developer
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#ifndef CUB_MAPPING__WAYPOINT_EDITOR_PANEL_HPP_
#define CUB_MAPPING__WAYPOINT_EDITOR_PANEL_HPP_

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <QMetaObject>
#include <QPointer>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "cub_mapping/srv/create_group.hpp"
#include "cub_mapping/srv/delete_group.hpp"
#include "cub_mapping/srv/get_group_list.hpp"
#include "cub_mapping/srv/load_waypoints.hpp"
#include "cub_mapping/srv/rename_group.hpp"
#include "cub_mapping/srv/set_require_input.hpp"

class QLabel;
class QPushButton;
class QTableWidget;
class QTableWidgetItem;
class QTimer;

namespace cub_mapping
{

/**
 * @brief RViz2 panel for managing waypoint groups and files
 *
 * Replaces the CLI service calls of waypoint_manager_node:
 * - Group list (name / waypoint count / require_input), refreshed every second
 * - Selecting a row sets the target group of WaypointEditTool
 * - Create / rename (double click) / delete groups, toggle require_input
 * - Save to the current file, load another YAML file
 */
class WaypointEditorPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit WaypointEditorPanel(QWidget * parent = nullptr);
  ~WaypointEditorPanel() override = default;

  void onInitialize() override;

private Q_SLOTS:
  void refresh();
  void onSelectionChanged();
  void onItemChanged(QTableWidgetItem * item);
  void onCreateGroup();
  void onRenameGroup();
  void onDeleteGroup();
  void onSave();
  void onLoad();

private:
  struct GroupInfo
  {
    std::string name;
    bool require_input;
    int waypoint_count;

    bool operator==(const GroupInfo & other) const
    {
      return name == other.name && require_input == other.require_input &&
             waypoint_count == other.waypoint_count;
    }
  };

  enum Column { COL_NAME = 0, COL_COUNT = 1, COL_REQUIRE_INPUT = 2 };

  void applyGroupList(const srv::GetGroupList::Response & response);
  void rebuildTable();
  void selectRow(int row);
  int selectedRow() const;
  void publishCurrentGroup();
  void setStatus(const QString & text, bool error = false);

  /// Call a service asynchronously and run on_result on the Qt thread
  template<typename ServiceT>
  void callService(
    const typename rclcpp::Client<ServiceT>::SharedPtr & client,
    const typename ServiceT::Request::SharedPtr & request,
    std::function<void(const typename ServiceT::Response &)> on_result)
  {
    if (!client || !client->service_is_ready()) {
      setStatus(
        QString("Service not available: %1 (is waypoint_manager_node running?)")
        .arg(client ? client->get_service_name() : "?"), true);
      return;
    }
    QPointer<WaypointEditorPanel> self(this);
    client->async_send_request(
      request,
      [self, on_result](typename rclcpp::Client<ServiceT>::SharedFuture future) {
        typename ServiceT::Response::SharedPtr response;
        try {
          response = future.get();
        } catch (const std::exception &) {
          return;
        }
        if (!self) {
          return;
        }
        QMetaObject::invokeMethod(
          self.data(), [self, on_result, response]() {
            if (self) {
              on_result(*response);
            }
          }, Qt::QueuedConnection);
      });
  }

  // ROS
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<srv::GetGroupList>::SharedPtr get_group_list_client_;
  rclcpp::Client<srv::CreateGroup>::SharedPtr create_group_client_;
  rclcpp::Client<srv::DeleteGroup>::SharedPtr delete_group_client_;
  rclcpp::Client<srv::RenameGroup>::SharedPtr rename_group_client_;
  rclcpp::Client<srv::SetRequireInput>::SharedPtr set_require_input_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr save_client_;
  rclcpp::Client<srv::LoadWaypoints>::SharedPtr load_client_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr current_group_pub_;

  // Widgets
  QLabel * file_label_;
  QPushButton * save_button_;
  QPushButton * load_button_;
  QTableWidget * table_;
  QPushButton * create_button_;
  QPushButton * rename_button_;
  QPushButton * delete_button_;
  QLabel * status_label_;
  QTimer * refresh_timer_;

  // State
  std::vector<GroupInfo> groups_;
  std::string file_path_;
  bool modified_;
  bool updating_table_;
  bool refresh_pending_;
  int refresh_wait_ticks_;
  int last_published_group_;
  int pending_select_row_;  // Row to select after the next refresh (-1 = none)
};

}  // namespace cub_mapping

#endif  // CUB_MAPPING__WAYPOINT_EDITOR_PANEL_HPP_
