#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <string>
#include <vector>
#include <memory>
#include <future>
#include <thread>

// 名前空間の省略
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

// クラス定義
class WaypointNavigator : public rclcpp::Node {
public:
  WaypointNavigator(const std::string & yaml_file)
  : Node("waypoint_navigator"),
    yaml_file_(yaml_file),
    current_group_index_(0)
  {
    // アクションクライアントの初期化
    navigate_to_pose_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

    // YAMLファイルの読み込み
    if (!load_yaml(yaml_file_)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load YAML file.");
      rclcpp::shutdown();
      return;
    }

    // ナビゲーションの開始
    start_navigation();
  }

private:
  std::string yaml_file_;
  std::vector<WaypointGroup> waypoint_groups_;
  size_t current_group_index_;

  rclcpp_action::Client<NavigateToPose>::SharedPtr navigate_to_pose_client_;

  // YAMLファイルの読み込み
  bool load_yaml(const std::string & filepath) {
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

  // ナビゲーションの開始
  void start_navigation() {
    if (current_group_index_ >= waypoint_groups_.size()) {
      RCLCPP_INFO(this->get_logger(), "All waypoint groups have been processed.");
      rclcpp::shutdown();
      return;
    }

    const WaypointGroup & current_group = waypoint_groups_[current_group_index_];
    RCLCPP_INFO(this->get_logger(), "Processing group: %s", current_group.group_name.c_str());

    // 各ウェイポイントを順番に処理
    for (size_t i = 0; i < current_group.waypoints.size(); ++i) {
      const Waypoint & wp = current_group.waypoints[i];
      RCLCPP_INFO(this->get_logger(), "Navigating to waypoint %zu: x=%.2f, y=%.2f, yaw=%.2f", 
                  i + 1, wp.x, wp.y, wp.yaw);
      
      // PoseStampedメッセージの作成
      auto pose = create_pose_stamped(wp);

      // アクションサーバーの待機
      if (!navigate_to_pose_client_->wait_for_action_server(10s)) {
        RCLCPP_ERROR(this->get_logger(), "NavigateToPose action server not available.");
        rclcpp::shutdown();
        return;
      }

      // ゴールメッセージの設定
      auto goal_msg = NavigateToPose::Goal();
      goal_msg.pose = pose;

      // アクションの送信と結果の待機
      auto send_goal_future = navigate_to_pose_client_->async_send_goal(goal_msg,
        rclcpp_action::Client<NavigateToPose>::SendGoalOptions());

      // ゴールの送信結果を待機
      if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), send_goal_future) 
          != rclcpp::FutureReturnCode::SUCCESS)
      {
        RCLCPP_ERROR(this->get_logger(), "Failed to send goal.");
        continue;
      }

      auto goal_handle = send_goal_future.get();
      if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server.");
        continue;
      }

      // 結果を待機
      auto result_future = navigate_to_pose_client_->async_get_result(goal_handle);
      if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) 
          != rclcpp::FutureReturnCode::SUCCESS)
      {
        RCLCPP_ERROR(this->get_logger(), "Failed to get result.");
        continue;
      }

      auto wrapped_result = result_future.get();
      switch (wrapped_result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          RCLCPP_INFO(this->get_logger(), "Navigation to waypoint %zu succeeded.", i + 1);
          break;
        case rclcpp_action::ResultCode::ABORTED:
          RCLCPP_ERROR(this->get_logger(), "Navigation to waypoint %zu aborted.", i + 1);
          continue;
        case rclcpp_action::ResultCode::CANCELED:
          RCLCPP_WARN(this->get_logger(), "Navigation to waypoint %zu canceled.", i + 1);
          continue;
        default:
          RCLCPP_ERROR(this->get_logger(), "Unknown result code for waypoint %zu.", i + 1);
          continue;
      }
    }

    // グループ全体でユーザ入力が要求されている場合、入力を待機
    if (current_group.require_input) {
      RCLCPP_INFO(this->get_logger(), "Waiting for user input to proceed to the next group...");
      wait_for_user_input();
    }

    // 次のグループへ進む
    current_group_index_++;
    start_navigation();
  }

  // PoseStampedメッセージの作成
  geometry_msgs::msg::PoseStamped create_pose_stamped(const Waypoint & wp) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = this->now();
    pose.pose.position.x = wp.x;
    pose.pose.position.y = wp.y;
    pose.pose.position.z = 0.0;

    // Yawからクォータニオンを計算
    double half_yaw = wp.yaw * 0.5;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = sin(half_yaw);
    pose.pose.orientation.w = cos(half_yaw);

    return pose;
  }

  // ユーザ入力の待機（別スレッドで実行）
  void wait_for_user_input() {
    std::future<void> future = std::async(std::launch::async, [&]() {
      std::cout << "Press Enter to continue..." << std::endl;
      std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    });

    // 入力待機
    future.wait();
  }
};

// メイン関数
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
