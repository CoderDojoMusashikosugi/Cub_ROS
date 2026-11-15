// Copyright 2025 Cub Developer
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "cub_mapping/waypoint_editor_panel.hpp"

#include <algorithm>

#include <QApplication>
#include <QFileDialog>
#include <QFileInfo>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QInputDialog>
#include <QLabel>
#include <QMessageBox>
#include <QPushButton>
#include <QTableWidget>
#include <QTimer>
#include <QVBoxLayout>

#include <rviz_common/display_context.hpp>

#include <pluginlib/class_list_macros.hpp>

#include "cub_mapping/waypoint_edit_tool.hpp"  // kCurrentGroupTopic

namespace cub_mapping
{

namespace
{
const QString kWaitingMessage = "Waiting for waypoint_manager_node...";
}  // namespace

WaypointEditorPanel::WaypointEditorPanel(QWidget * parent)
: rviz_common::Panel(parent),
  modified_(false),
  updating_table_(false),
  refresh_pending_(false),
  refresh_wait_ticks_(0),
  last_published_group_(-1),
  pending_select_row_(-1)
{
  // File row
  file_label_ = new QLabel("File: (none)");
  file_label_->setWordWrap(true);
  save_button_ = new QPushButton("Save");
  load_button_ = new QPushButton("Load...");

  auto * file_buttons = new QHBoxLayout;
  file_buttons->addWidget(save_button_);
  file_buttons->addWidget(load_button_);

  // Group table
  table_ = new QTableWidget(0, 3);
  table_->setHorizontalHeaderLabels({"Group", "WPs", "Require Input"});
  table_->horizontalHeader()->setSectionResizeMode(COL_NAME, QHeaderView::Stretch);
  table_->horizontalHeader()->setSectionResizeMode(COL_COUNT, QHeaderView::ResizeToContents);
  table_->horizontalHeader()->setSectionResizeMode(
    COL_REQUIRE_INPUT, QHeaderView::ResizeToContents);
  table_->verticalHeader()->setVisible(true);
  table_->setSelectionBehavior(QAbstractItemView::SelectRows);
  table_->setSelectionMode(QAbstractItemView::SingleSelection);
  table_->setEditTriggers(QAbstractItemView::DoubleClicked | QAbstractItemView::EditKeyPressed);

  // Group buttons
  create_button_ = new QPushButton("New Group");
  rename_button_ = new QPushButton("Rename");
  delete_button_ = new QPushButton("Delete");
  auto * group_buttons = new QHBoxLayout;
  group_buttons->addWidget(create_button_);
  group_buttons->addWidget(rename_button_);
  group_buttons->addWidget(delete_button_);

  status_label_ = new QLabel;
  status_label_->setWordWrap(true);

  auto * layout = new QVBoxLayout;
  layout->addWidget(file_label_);
  layout->addLayout(file_buttons);
  layout->addWidget(table_, 1);
  layout->addLayout(group_buttons);
  layout->addWidget(status_label_);
  setLayout(layout);

  connect(save_button_, &QPushButton::clicked, this, &WaypointEditorPanel::onSave);
  connect(load_button_, &QPushButton::clicked, this, &WaypointEditorPanel::onLoad);
  connect(create_button_, &QPushButton::clicked, this, &WaypointEditorPanel::onCreateGroup);
  connect(rename_button_, &QPushButton::clicked, this, &WaypointEditorPanel::onRenameGroup);
  connect(delete_button_, &QPushButton::clicked, this, &WaypointEditorPanel::onDeleteGroup);
  connect(table_, &QTableWidget::itemSelectionChanged,
          this, &WaypointEditorPanel::onSelectionChanged);
  connect(table_, &QTableWidget::itemChanged, this, &WaypointEditorPanel::onItemChanged);

  refresh_timer_ = new QTimer(this);
  connect(refresh_timer_, &QTimer::timeout, this, &WaypointEditorPanel::refresh);
}

void WaypointEditorPanel::onInitialize()
{
  node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  get_group_list_client_ = node_->create_client<srv::GetGroupList>("get_group_list");
  create_group_client_ = node_->create_client<srv::CreateGroup>("create_group");
  delete_group_client_ = node_->create_client<srv::DeleteGroup>("delete_group");
  rename_group_client_ = node_->create_client<srv::RenameGroup>("rename_group");
  set_require_input_client_ =
    node_->create_client<srv::SetRequireInput>("set_require_input");
  save_client_ = node_->create_client<std_srvs::srv::Trigger>("save_waypoints");
  load_client_ = node_->create_client<srv::LoadWaypoints>("load_waypoints");

  current_group_pub_ = node_->create_publisher<std_msgs::msg::Int32>(
    kCurrentGroupTopic, rclcpp::QoS(1).transient_local());

  refresh_timer_->start(1000);
  refresh();
}

// ---------------------------------------------------------------------------
// Group list synchronization
// ---------------------------------------------------------------------------

void WaypointEditorPanel::refresh()
{
  if (!get_group_list_client_) {
    return;
  }
  if (refresh_pending_) {
    // Give up on a request that never got a response (e.g. node restarted)
    if (++refresh_wait_ticks_ < 3) {
      return;
    }
    refresh_pending_ = false;
  }
  if (!get_group_list_client_->service_is_ready()) {
    setStatus(kWaitingMessage, true);
    return;
  }

  refresh_pending_ = true;
  refresh_wait_ticks_ = 0;
  callService<srv::GetGroupList>(
    get_group_list_client_, std::make_shared<srv::GetGroupList::Request>(),
    [this](const srv::GetGroupList::Response & response) {
      refresh_pending_ = false;
      // Connected: clear the "waiting" message shown before the node came up
      if (status_label_->text() == kWaitingMessage) {
        setStatus("");
      }
      applyGroupList(response);
    });
}

void WaypointEditorPanel::applyGroupList(const srv::GetGroupList::Response & response)
{
  std::vector<GroupInfo> groups;
  for (size_t i = 0; i < response.group_names.size(); ++i) {
    GroupInfo info;
    info.name = response.group_names[i];
    info.require_input =
      i < response.require_input_flags.size() && response.require_input_flags[i];
    info.waypoint_count =
      i < response.waypoint_counts.size() ? response.waypoint_counts[i] : 0;
    groups.push_back(info);
  }

  // File label
  file_path_ = response.file_path;
  modified_ = response.modified;
  QString file_text = file_path_.empty() ?
    QString("File: (none)") :
    QString("File: %1").arg(QFileInfo(QString::fromStdString(file_path_)).fileName());
  if (modified_) {
    file_text += "  [unsaved changes]";
  }
  file_label_->setText(file_text);
  file_label_->setToolTip(QString::fromStdString(file_path_));
  file_label_->setStyleSheet(modified_ ? "color: #d97706; font-weight: bold;" : "");

  // Don't disturb the user while a group name is being edited
  // (an inline editor widget inside the table has the focus while editing)
  QWidget * focus = QApplication::focusWidget();
  if (focus && focus != table_ && table_->isAncestorOf(focus)) {
    return;
  }

  if (groups != groups_ || table_->rowCount() != static_cast<int>(groups.size())) {
    groups_ = groups;
    rebuildTable();
  }

  if (pending_select_row_ >= 0) {
    selectRow(pending_select_row_);
    pending_select_row_ = -1;
  } else if (selectedRow() < 0 && !groups_.empty()) {
    selectRow(0);
  }

  publishCurrentGroup();
}

void WaypointEditorPanel::rebuildTable()
{
  int previous_row = selectedRow();

  updating_table_ = true;
  table_->setRowCount(static_cast<int>(groups_.size()));
  for (size_t i = 0; i < groups_.size(); ++i) {
    const auto & g = groups_[i];
    int row = static_cast<int>(i);

    auto * name_item = new QTableWidgetItem(QString::fromStdString(g.name));
    name_item->setFlags(Qt::ItemIsSelectable | Qt::ItemIsEnabled | Qt::ItemIsEditable);
    table_->setItem(row, COL_NAME, name_item);

    auto * count_item = new QTableWidgetItem(QString::number(g.waypoint_count));
    count_item->setFlags(Qt::ItemIsSelectable | Qt::ItemIsEnabled);
    count_item->setTextAlignment(Qt::AlignCenter);
    table_->setItem(row, COL_COUNT, count_item);

    auto * input_item = new QTableWidgetItem;
    input_item->setFlags(Qt::ItemIsSelectable | Qt::ItemIsEnabled | Qt::ItemIsUserCheckable);
    input_item->setCheckState(g.require_input ? Qt::Checked : Qt::Unchecked);
    table_->setItem(row, COL_REQUIRE_INPUT, input_item);

    // Row numbers = group indices (0-based, same as marker namespaces)
    table_->setVerticalHeaderItem(row, new QTableWidgetItem(QString::number(row)));
  }
  updating_table_ = false;

  if (previous_row >= 0) {
    selectRow(std::min(previous_row, static_cast<int>(groups_.size()) - 1));
  }
}

void WaypointEditorPanel::selectRow(int row)
{
  if (row < 0 || row >= table_->rowCount()) {
    return;
  }
  table_->selectRow(row);
}

int WaypointEditorPanel::selectedRow() const
{
  auto rows = table_->selectionModel()->selectedRows();
  return rows.isEmpty() ? -1 : rows.first().row();
}

void WaypointEditorPanel::publishCurrentGroup()
{
  if (!current_group_pub_) {
    return;
  }
  int row = selectedRow();
  if (row < 0) {
    row = 0;
  }
  if (row == last_published_group_) {
    return;
  }
  std_msgs::msg::Int32 msg;
  msg.data = row;
  current_group_pub_->publish(msg);
  last_published_group_ = row;
}

void WaypointEditorPanel::onSelectionChanged()
{
  if (updating_table_) {
    return;
  }
  publishCurrentGroup();
}

// ---------------------------------------------------------------------------
// Group operations
// ---------------------------------------------------------------------------

void WaypointEditorPanel::onItemChanged(QTableWidgetItem * item)
{
  if (updating_table_ || !item) {
    return;
  }
  int row = item->row();
  if (row < 0 || row >= static_cast<int>(groups_.size())) {
    return;
  }

  if (item->column() == COL_NAME) {
    std::string new_name = item->text().trimmed().toStdString();
    if (new_name.empty() || new_name == groups_[row].name) {
      // Revert empty / unchanged names
      updating_table_ = true;
      item->setText(QString::fromStdString(groups_[row].name));
      updating_table_ = false;
      return;
    }
    auto request = std::make_shared<srv::RenameGroup::Request>();
    request->group_index = row;
    request->new_name = new_name;
    callService<srv::RenameGroup>(
      rename_group_client_, request,
      [this](const srv::RenameGroup::Response & response) {
        setStatus(QString::fromStdString(response.message), !response.success);
        refresh();
      });
  } else if (item->column() == COL_REQUIRE_INPUT) {
    bool checked = item->checkState() == Qt::Checked;
    if (checked == groups_[row].require_input) {
      return;
    }
    auto request = std::make_shared<srv::SetRequireInput::Request>();
    request->group_index = row;
    request->require_input = checked;
    callService<srv::SetRequireInput>(
      set_require_input_client_, request,
      [this](const srv::SetRequireInput::Response & response) {
        setStatus(QString::fromStdString(response.message), !response.success);
        refresh();
      });
  }
}

void WaypointEditorPanel::onCreateGroup()
{
  bool ok = false;
  QString default_name = QString("Group %1").arg(groups_.size());
  QString name = QInputDialog::getText(
    this, "New Group", "Group name:", QLineEdit::Normal, default_name, &ok).trimmed();
  if (!ok || name.isEmpty()) {
    return;
  }

  auto request = std::make_shared<srv::CreateGroup::Request>();
  request->group_name = name.toStdString();
  request->require_input = false;
  callService<srv::CreateGroup>(
    create_group_client_, request,
    [this](const srv::CreateGroup::Response & response) {
      setStatus(QString::fromStdString(response.message), !response.success);
      if (response.success) {
        // Make the new group the target for adding waypoints
        pending_select_row_ = response.group_index;
      }
      refresh();
    });
}

void WaypointEditorPanel::onRenameGroup()
{
  int row = selectedRow();
  if (row < 0) {
    setStatus("Select a group to rename", true);
    return;
  }
  table_->editItem(table_->item(row, COL_NAME));
}

void WaypointEditorPanel::onDeleteGroup()
{
  int row = selectedRow();
  if (row < 0 || row >= static_cast<int>(groups_.size())) {
    setStatus("Select a group to delete", true);
    return;
  }

  const auto & g = groups_[row];
  auto answer = QMessageBox::question(
    this, "Delete Group",
    QString("Delete group [%1] \"%2\" and its %3 waypoint(s)?")
    .arg(row).arg(QString::fromStdString(g.name)).arg(g.waypoint_count),
    QMessageBox::Yes | QMessageBox::No, QMessageBox::No);
  if (answer != QMessageBox::Yes) {
    return;
  }

  auto request = std::make_shared<srv::DeleteGroup::Request>();
  request->group_index = row;
  callService<srv::DeleteGroup>(
    delete_group_client_, request,
    [this, row](const srv::DeleteGroup::Response & response) {
      setStatus(QString::fromStdString(response.message), !response.success);
      if (response.success) {
        // Keep the selection near the deleted row
        pending_select_row_ = std::max(0, row - 1);
      }
      refresh();
    });
}

// ---------------------------------------------------------------------------
// File operations
// ---------------------------------------------------------------------------

void WaypointEditorPanel::onSave()
{
  callService<std_srvs::srv::Trigger>(
    save_client_, std::make_shared<std_srvs::srv::Trigger::Request>(),
    [this](const std_srvs::srv::Trigger::Response & response) {
      setStatus(QString::fromStdString(response.message), !response.success);
      refresh();
    });
}

void WaypointEditorPanel::onLoad()
{
  if (modified_) {
    auto answer = QMessageBox::question(
      this, "Load Waypoints",
      "There are unsaved changes. Discard them and load another file?",
      QMessageBox::Yes | QMessageBox::No, QMessageBox::No);
    if (answer != QMessageBox::Yes) {
      return;
    }
  }

  QString start_dir = file_path_.empty() ?
    QString() : QFileInfo(QString::fromStdString(file_path_)).absolutePath();
  QString path = QFileDialog::getOpenFileName(
    this, "Load Waypoints", start_dir, "Waypoint YAML (*.yaml *.yml);;All files (*)");
  if (path.isEmpty()) {
    return;
  }

  auto request = std::make_shared<srv::LoadWaypoints::Request>();
  request->filepath = path.toStdString();
  callService<srv::LoadWaypoints>(
    load_client_, request,
    [this](const srv::LoadWaypoints::Response & response) {
      setStatus(QString::fromStdString(response.message), !response.success);
      if (response.success) {
        pending_select_row_ = 0;
      }
      refresh();
    });
}

void WaypointEditorPanel::setStatus(const QString & text, bool error)
{
  if (text == status_label_->text()) {
    return;  // Avoid flooding the log with repeated messages
  }
  status_label_->setText(text);
  status_label_->setStyleSheet(error ? "color: #dc2626;" : "");
  if (node_ && !text.isEmpty()) {
    if (error) {
      RCLCPP_WARN(node_->get_logger(), "[WaypointEditorPanel] %s", text.toStdString().c_str());
    } else {
      RCLCPP_INFO(node_->get_logger(), "[WaypointEditorPanel] %s", text.toStdString().c_str());
    }
  }
}

}  // namespace cub_mapping

PLUGINLIB_EXPORT_CLASS(cub_mapping::WaypointEditorPanel, rviz_common::Panel)
