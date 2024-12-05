#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <yaml-cpp/yaml.h>
#include <std_msgs/msg/empty.hpp>
#include <fstream>
#include <string>
#include <vector>
#include <memory>
#include <mutex>
#include <condition_variable>

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using namespace std::chrono_literals;

// ウェイポイントの構造体
struct Waypoint {
  double x;
  double y;
  double yaw;
};

// グループの構造体
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
    is_waiting_for_input_(false)
  {
    // アクションクライアントの初期化
    navigate_to_pose_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

    // サブスクリプションの設定
    proceed_subscription_ = this->create_subscription<std_msgs::msg::Empty>(
      "/proceed_to_next_group",
      10,
      std::bind(&WaypointNavigator::proceed_callback, this, std::placeholders::_1)
    );

    // YAMLファイルの読み込み
    if (!load_yaml(yaml_file_)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load YAML file.");
      rclcpp::shutdown();
      return;
    }

    // ナビゲーションの開始
    send_next_goal();
  }

private:
  std::string yaml_file_;
  std::vector<WaypointGroup> waypoint_groups_;
  size_t current_group_index_;
  size_t current_waypoint_index_;
  bool is_waiting_for_input_;

  rclcpp_action::Client<NavigateToPose>::SharedPtr navigate_to_pose_client_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr proceed_subscription_;

  std::mutex mutex_;
  std::condition_variable cv_;

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

    // 現在のグループのウェイポイントを全て処理した場合
    if (current_waypoint_index_ >= current_group.waypoints.size()) {
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
      return;
    }

    // 現在のウェイポイントを取得
    const Waypoint & wp = current_group.waypoints[current_waypoint_index_];
    RCLCPP_INFO(this->get_logger(), "Navigating to waypoint %zu in group '%s': x=%.2f, y=%.2f, yaw=%.2f", 
                current_waypoint_index_ + 1, current_group.group_name.c_str(), wp.x, wp.y, wp.yaw);

    // PoseStampedメッセージの作成
    auto pose = create_pose_stamped(wp);

    // アクションサーバーが利用可能か確認
    if (!navigate_to_pose_client_->wait_for_action_server(10s)) {
      RCLCPP_ERROR(this->get_logger(), "NavigateToPose action server not available.");
      rclcpp::shutdown();
      return;
    }

    // ゴールメッセージの設定
    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose = pose;

    // SendGoalOptions を設定
    rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
    options.goal_response_callback = std::bind(&WaypointNavigator::goal_response_callback, this, std::placeholders::_1);
    options.result_callback = std::bind(&WaypointNavigator::result_callback, this, std::placeholders::_1);

    // アクションの送信
    navigate_to_pose_client_->async_send_goal(goal_msg, options);
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

  void goal_response_callback(rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by the server.");
      // 次のウェイポイントへ進む
      current_waypoint_index_++;
      send_next_goal();
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Goal accepted by the server, waiting for result...");
    // 結果の処理は result_callback に委譲
  }

  void result_callback(const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult & result)
  {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_INFO(this->get_logger(), "Navigation to waypoint %zu succeeded.", current_waypoint_index_ + 1);
    } else if (result.code == rclcpp_action::ResultCode::ABORTED) {
      RCLCPP_ERROR(this->get_logger(), "Navigation to waypoint %zu aborted.", current_waypoint_index_ + 1);
    } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
      RCLCPP_WARN(this->get_logger(), "Navigation to waypoint %zu canceled.", current_waypoint_index_ + 1);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Unknown result code for waypoint %zu.", current_waypoint_index_ + 1);
    }

    // 次のウェイポイントへ進む
    current_waypoint_index_++;
    send_next_goal();
  }

  void proceed_callback(const std_msgs::msg::Empty::SharedPtr msg)
  {
    if (!is_waiting_for_input_) {
      RCLCPP_WARN(this->get_logger(), "Received 'proceed_to_next_group' message but not waiting for input.");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Received 'proceed_to_next_group' message. Proceeding to the next group.");

    // 入力待機フラグをリセット
    is_waiting_for_input_ = false;

    // 次のグループへ進む
    current_group_index_++;
    current_waypoint_index_ = 0;
    send_next_goal();
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

  // ノードを作成
  auto node = std::make_shared<WaypointNavigator>(yaml_file);

  // シングルスレッドエグゼキュータでスピン
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
