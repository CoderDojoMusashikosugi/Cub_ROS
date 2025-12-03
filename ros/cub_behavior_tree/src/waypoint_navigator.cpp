#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/action/navigate_through_poses.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <yaml-cpp/yaml.h>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/bool.hpp>
#include <fstream>
#include <string>
#include <vector>
#include <memory>
#include <mutex>
#include <condition_variable>

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using NavigateThroughPoses = nav2_msgs::action::NavigateThroughPoses;
using namespace std::chrono_literals;

struct Waypoint {
  double x;
  double y;
  double yaw;
};

struct WaypointGroup {
  std::string group_name;
  bool require_input;
  std::vector<Waypoint> waypoints;
};

class WaypointNavigator : public rclcpp::Node
{
public:
  WaypointNavigator(const std::string & yaml_file)
  : Node("waypoint_navigator"),
    yaml_file_(yaml_file),
    current_group_index_(0),
    current_waypoint_index_(0),
    is_waiting_for_input_(true), // 起動時にユーザー入力待ちにする
    follow_mode_enabled_(false)
  {
    navigate_to_pose_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
    navigate_through_poses_client_ = rclcpp_action::create_client<NavigateThroughPoses>(this, "navigate_through_poses");

    proceed_subscription_ = this->create_subscription<std_msgs::msg::Empty>(
      "/proceed_to_next_group",
      10,
      std::bind(&WaypointNavigator::proceed_callback, this, std::placeholders::_1)
    );

    follow_mode_pub_ = this->create_publisher<std_msgs::msg::Bool>(
      "/follow_mode_enabled", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local());
    publish_follow_mode(false);

    if (!load_yaml(yaml_file_)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load YAML file.");
      rclcpp::shutdown();
      return;
    }

    // Node起動時は自動的に目標を送らず、ユーザ入力を待つ
    RCLCPP_INFO(this->get_logger(), "Node started. Waiting for 'proceed_to_next_group' to begin navigation.");
  }

private:
  std::string yaml_file_;
  std::vector<WaypointGroup> waypoint_groups_;
  size_t current_group_index_;
  size_t current_waypoint_index_;
  bool is_waiting_for_input_;

  rclcpp_action::Client<NavigateToPose>::SharedPtr navigate_to_pose_client_;
  rclcpp_action::Client<NavigateThroughPoses>::SharedPtr navigate_through_poses_client_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr proceed_subscription_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr follow_mode_pub_;

  std::mutex mutex_;
  std::condition_variable cv_;

  bool follow_mode_enabled_;

  void publish_follow_mode(bool enabled)
  {
    if (follow_mode_enabled_ == enabled) {
      return;
    }

    follow_mode_enabled_ = enabled;
    std_msgs::msg::Bool msg;
    msg.data = enabled;
    follow_mode_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Front follow mode %s", enabled ? "ENABLED" : "DISABLED");
  }

  bool load_yaml(const std::string & filepath)
  {
    try {
      YAML::Node config = YAML::LoadFile(filepath);
      if (!config["waypoint_groups"]) {
        RCLCPP_ERROR(this->get_logger(), "YAML file does not contain 'waypoint_groups'.");
        return false;
      }

      for (const auto & group_node : config["waypoint_groups"]) {
        WaypointGroup group;
        group.group_name = group_node["group_name"].as<std::string>();
        group.require_input = group_node["require_input"].as<bool>();

        for (const auto & wp_node : group_node["waypoints"]) {
          Waypoint wp;
          wp.x = wp_node["x"].as<double>();
          wp.y = wp_node["y"].as<double>();
          wp.yaw = wp_node["yaw"].as<double>();
          group.waypoints.push_back(wp);
        }

        waypoint_groups_.push_back(group);
      }

      return true;
    }
    catch (const YAML::Exception & e) {
      RCLCPP_ERROR(this->get_logger(), "YAML Exception: %s", e.what());
      return false;
    }
    catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "Exception: %s", e.what());
      return false;
    }
  }

  void send_next_goal()
  {
    // グループの範囲外に達した場合
    if (current_group_index_ >= waypoint_groups_.size()) {
      RCLCPP_INFO(this->get_logger(), "All waypoint groups have been processed.");
      rclcpp::shutdown();
      return;
    }

    WaypointGroup & current_group = waypoint_groups_[current_group_index_];

    if (current_waypoint_index_ >= current_group.waypoints.size()) {
      if (current_group.require_input) {
        publish_follow_mode(false);
        RCLCPP_INFO(this->get_logger(), "Group '%s' requires input. Waiting for 'proceed_to_next_group' message.", 
                    current_group.group_name.c_str());
        is_waiting_for_input_ = true;
      } else {
        RCLCPP_INFO(this->get_logger(), "Group '%s' does not require input. Proceeding to the next group.", 
                    current_group.group_name.c_str());
        current_group_index_++;
        current_waypoint_index_ = 0;
        send_next_goal();
      }
      return;
    }

    const bool is_last_in_group = current_waypoint_index_ == current_group.waypoints.size() - 1;
    const bool enable_follow_mode = current_group.require_input && is_last_in_group;

    if (current_group.require_input) {
      send_navigate_to_pose(current_group.waypoints[current_waypoint_index_], enable_follow_mode);
      return;
    }

    publish_follow_mode(false);
    if (current_group.waypoints.size() - current_waypoint_index_ > 1) {
      send_navigate_through_poses(current_group);
    } else {
      send_navigate_to_pose(current_group.waypoints[current_waypoint_index_], false);
    }
  }

  void send_navigate_to_pose(const Waypoint & wp, bool enable_follow_mode)
  {
    RCLCPP_INFO(this->get_logger(), "Navigating to single waypoint in group '%s': x=%.2f, y=%.2f, yaw=%.2f", 
                waypoint_groups_[current_group_index_].group_name.c_str(), wp.x, wp.y, wp.yaw);

    auto pose = create_pose_stamped(wp);

    if (!navigate_to_pose_client_->wait_for_action_server(10s)) {
      RCLCPP_ERROR(this->get_logger(), "NavigateToPose action server not available.");
      rclcpp::shutdown();
      return;
    }

    publish_follow_mode(enable_follow_mode);

    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose = pose;

    rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
    options.goal_response_callback = 
      std::bind(&WaypointNavigator::navigate_to_pose_goal_response_callback, this, std::placeholders::_1);
    options.result_callback = 
      std::bind(&WaypointNavigator::navigate_to_pose_result_callback, this, std::placeholders::_1);

    navigate_to_pose_client_->async_send_goal(goal_msg, options);
  }

  void send_navigate_through_poses(WaypointGroup & group)
  {
    size_t remaining_waypoints = group.waypoints.size() - current_waypoint_index_;
    RCLCPP_INFO(this->get_logger(), "Navigating through %zu waypoints in group '%s'.", 
                remaining_waypoints, group.group_name.c_str());

    std::vector<geometry_msgs::msg::PoseStamped> poses;
    for (size_t i = current_waypoint_index_; i < group.waypoints.size(); ++i) {
      const Waypoint & wp = group.waypoints[i];
      geometry_msgs::msg::PoseStamped pose = create_pose_stamped(wp);
      poses.push_back(pose);
    }

    if (!navigate_through_poses_client_->wait_for_action_server(10s)) {
      RCLCPP_ERROR(this->get_logger(), "NavigateThroughPoses action server not available.");
      rclcpp::shutdown();
      return;
    }

    publish_follow_mode(false);
    auto goal_msg = NavigateThroughPoses::Goal();
    goal_msg.poses = poses;

    rclcpp_action::Client<NavigateThroughPoses>::SendGoalOptions options;
    options.goal_response_callback = 
      std::bind(&WaypointNavigator::navigate_through_poses_goal_response_callback, this, std::placeholders::_1);
    options.result_callback = 
      std::bind(&WaypointNavigator::navigate_through_poses_result_callback, this, std::placeholders::_1);

    navigate_through_poses_client_->async_send_goal(goal_msg, options);
  }

  geometry_msgs::msg::PoseStamped create_pose_stamped(const Waypoint & wp)
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = this->now();
    pose.pose.position.x = wp.x;
    pose.pose.position.y = wp.y;
    pose.pose.position.z = 0.0;

    double half_yaw = wp.yaw * 0.5;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = sin(half_yaw);
    pose.pose.orientation.w = cos(half_yaw);

    return pose;
  }

  void navigate_to_pose_goal_response_callback(rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "NavigateToPose goal was rejected by the server.");
      publish_follow_mode(false);
      current_waypoint_index_++;
      send_next_goal();
      return;
    }

    RCLCPP_INFO(this->get_logger(), "NavigateToPose goal accepted by the server, waiting for result...");
  }

  void navigate_to_pose_result_callback(const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult & result)
  {
    publish_follow_mode(false);

    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_INFO(this->get_logger(), "Navigation to waypoint %zu succeeded.", current_waypoint_index_ + 1);
    } else if (result.code == rclcpp_action::ResultCode::ABORTED) {
      RCLCPP_ERROR(this->get_logger(), "Navigation to waypoint %zu aborted.", current_waypoint_index_ + 1);
    } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
      RCLCPP_WARN(this->get_logger(), "Navigation to waypoint %zu canceled.", current_waypoint_index_ + 1);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Unknown result code for waypoint %zu.", current_waypoint_index_ + 1);
    }

    current_waypoint_index_++;
    send_next_goal();
  }

  void navigate_through_poses_goal_response_callback(rclcpp_action::ClientGoalHandle<NavigateThroughPoses>::SharedPtr goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "NavigateThroughPoses goal was rejected by the server.");
      current_group_index_++;
      current_waypoint_index_ = 0;
      publish_follow_mode(false);
      send_next_goal();
      return;
    }

    RCLCPP_INFO(this->get_logger(), "NavigateThroughPoses goal accepted by the server, waiting for result...");
  }

  void navigate_through_poses_result_callback(const rclcpp_action::ClientGoalHandle<NavigateThroughPoses>::WrappedResult & result)
  {
    publish_follow_mode(false);

    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_INFO(this->get_logger(), "Navigation through group '%s' succeeded.", 
                  waypoint_groups_[current_group_index_].group_name.c_str());
    } else if (result.code == rclcpp_action::ResultCode::ABORTED) {
      RCLCPP_ERROR(this->get_logger(), "Navigation through group '%s' aborted.", 
                   waypoint_groups_[current_group_index_].group_name.c_str());
      RCLCPP_INFO(this->get_logger(), "retry this groups");
      send_next_goal();
      return;
    } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
      RCLCPP_WARN(this->get_logger(), "Navigation through group '%s' canceled.", 
                  waypoint_groups_[current_group_index_].group_name.c_str());
    } else {
      RCLCPP_ERROR(this->get_logger(), "Unknown result code for navigation through group '%s'.", 
                   waypoint_groups_[current_group_index_].group_name.c_str());
    }

    WaypointGroup & current_group = waypoint_groups_[current_group_index_];
    if (current_group.require_input) {
      RCLCPP_INFO(this->get_logger(), "Group '%s' requires input. Waiting for 'proceed_to_next_group' message.", 
                  current_group.group_name.c_str());
      is_waiting_for_input_ = true;
    } else {
      RCLCPP_INFO(this->get_logger(), "Group '%s' does not require input. Proceeding to the next group.", 
                  current_group.group_name.c_str());
      current_group_index_++;
      current_waypoint_index_ = 0;
      send_next_goal();
    }
  }

  void proceed_callback(const std_msgs::msg::Empty::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!is_waiting_for_input_) {
      RCLCPP_WARN(this->get_logger(), "Received 'proceed_to_next_group' message but not waiting for input.");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Received 'proceed_to_next_group' message. Proceeding...");

    is_waiting_for_input_ = false;
    publish_follow_mode(false);

    // もし最初のグループにまだ着手していない場合、ここでスタート
    if (current_group_index_ == 0 && current_waypoint_index_ == 0) {
      send_next_goal();
    } else {
      // 現在停止中の状態から次のグループへ進む
      current_group_index_++;
      current_waypoint_index_ = 0;
      send_next_goal();
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  if (argc < 2) {
    std::cerr << "Usage: waypoint_navigator <path_to_yaml_file>" << std::endl;
    return 1;
  }

  std::string yaml_file = argv[1];

  auto node = std::make_shared<WaypointNavigator>(yaml_file);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
