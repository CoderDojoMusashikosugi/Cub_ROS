#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/msg/speed_limit.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <yaml-cpp/yaml.h>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/bool.hpp>
#include <fstream>
#include <cmath>
#include <limits>
#include <string>
#include <vector>
#include <memory>
#include <mutex>
#include <condition_variable>
#include <cstdint>

using NavigateToPose = nav2_msgs::action::NavigateToPose;
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
    // 停止地点以外のwaypointでは、Nav2の到達判定を待たずに手前で次のゴールを送って上書き(preempt)する。
    // ゴール完了時にcontroller_serverがMPPIをリセットして減速・停止するのを避けるため。
    // Nav2の到達判定(goal checkerのxy_goal_tolerance)はnav2のパラメータファイルで停止地点向けの値に固定する。
    enable_pass_through_ = this->declare_parameter<bool>("enable_pass_through", true);
    // 通過扱いのwaypointまでの直線距離がこの値[m]を切ったら次のwaypointへ切り替える
    pass_through_distance_ = this->declare_parameter<double>("pass_through_distance", 3.0);
    // 最後のwaypoint(停止地点)へ向かうときの速度上限[m/s]。0以下で無効。
    final_approach_speed_ = this->declare_parameter<double>("final_approach_speed", 0.3);
    // 最後のwaypointまでの直線距離がこの値[m]を切ったら速度を落とす
    final_approach_distance_ = this->declare_parameter<double>("final_approach_distance", 3.0);
    speed_limit_pub_ = this->create_publisher<nav2_msgs::msg::SpeedLimit>("/speed_limit", 10);

    navigate_to_pose_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

    proceed_subscription_ = this->create_subscription<std_msgs::msg::Empty>(
      "/proceed_to_next_group",
      10,
      std::bind(&WaypointNavigator::proceed_callback, this, std::placeholders::_1)
    );

    follow_mode_pub_ = this->create_publisher<std_msgs::msg::Bool>(
      "/follow_mode_enabled", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local());
    publish_follow_mode(false);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

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
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr proceed_subscription_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr follow_mode_pub_;
  rclcpp::Publisher<nav2_msgs::msg::SpeedLimit>::SharedPtr speed_limit_pub_;
  double final_approach_speed_{0.3};  // require_inputグループの最後のwaypoint(停止地点)に近づいたときの速度上限[m/s]。0以下で無効。
  double final_approach_distance_{2.0}; // require_inputグループの最後のwaypoint(停止地点)までの直線距離がこの値[m]を切ったら速度を落とす
  bool final_approach_slowdown_active_{false};
  rclcpp::TimerBase::SharedPtr resend_timer_;

  std::mutex mutex_;
  std::condition_variable cv_;

  bool follow_mode_enabled_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  Waypoint resend_waypoint_;
  bool has_resend_waypoint_{false};
  bool resend_follow_mode_{false};

  bool enable_pass_through_{true};
  double pass_through_distance_{3.0};
  // 送信したゴールの通し番号。上書きされた古いゴールのコールバックを無視するために使う。
  uint64_t goal_seq_{0};

  enum class ResendDecision
  {
    None,
    ResendWithFollow,
    ResendWithoutFollow
  };

  void publish_follow_mode(bool enabled, bool force = false)
  {
    const bool changed = follow_mode_enabled_ != enabled;
    if (!force && !changed) {
      return;
    }
    follow_mode_enabled_ = enabled;
    std_msgs::msg::Bool msg;
    msg.data = enabled;
    follow_mode_pub_->publish(msg);
    if (changed) {
      RCLCPP_INFO(this->get_logger(), "Front follow mode %s", enabled ? "ENABLED" : "DISABLED");
    }
  }

  // 現在のwaypointが入力待ちの停止地点(require_inputグループの最後)か。減速対象はここだけ。
  bool is_final_waypoint() const
  {
    if (current_group_index_ >= waypoint_groups_.size()) {
      return false;
    }
    const WaypointGroup & group = waypoint_groups_[current_group_index_];
    return group.require_input && current_waypoint_index_ + 1 == group.waypoints.size();
  }

  // controller_serverの速度上限を設定する。limit<=0で上限解除。
  // MPPIはゴール終了時のリセットで速度上限が解除されるため、ゴール受理のたびに送り直す。
  void publish_speed_limit(double limit)
  {
    nav2_msgs::msg::SpeedLimit msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "map";
    msg.percentage = false;
    msg.speed_limit = limit > 0.0 ? limit : 0.0;  // 0.0はNav2で「上限なし」
    speed_limit_pub_->publish(msg);
    if (limit > 0.0) {
      RCLCPP_INFO(this->get_logger(), "Speed limit set to %.2f m/s for the final waypoint", limit);
    }
  }

  // 到達を待たずに次のwaypointへ上書きしてよいwaypointか(入力待ちの停止地点・ルート最後以外)
  bool is_pass_through_waypoint() const
  {
    if (current_group_index_ >= waypoint_groups_.size()) {
      return false;
    }
    const WaypointGroup & group = waypoint_groups_[current_group_index_];
    if (current_waypoint_index_ >= group.waypoints.size()) {
      return false;
    }
    if (current_waypoint_index_ + 1 < group.waypoints.size()) {
      return true;
    }
    if (group.require_input) {
      return false;
    }
    // 入力不要グループの最後: 後続グループに続きのwaypointがあれば通過扱い(空グループは読み飛ばす)。
    // ルート全体の最後なら通過扱いにせず、Nav2の到達判定(xy_goal_tolerance)でゴールさせる。
    for (size_t i = current_group_index_ + 1; i < waypoint_groups_.size(); ++i) {
      if (!waypoint_groups_[i].waypoints.empty()) {
        return true;
      }
    }
    return false;
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

    // 追従モードは使わず、全waypointを通常のナビゲーションで走る
    publish_follow_mode(false);
    send_navigate_to_pose(current_group.waypoints[current_waypoint_index_], false);
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

    publish_follow_mode(enable_follow_mode, true);

    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose = pose;

    // 上書き(preempt)された古いゴールのコールバックは通し番号で無視する
    const uint64_t seq = ++goal_seq_;

    rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
    options.goal_response_callback =
      [this, seq](rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr goal_handle) {
        if (seq == goal_seq_) {
          navigate_to_pose_goal_response_callback(goal_handle);
        }
      };
    options.feedback_callback =
      [this, seq](
      rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr,
      const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
        if (seq == goal_seq_) {
          navigate_to_pose_feedback_callback(feedback);
        }
      };
    options.result_callback =
      [this, seq](const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult & result) {
        if (seq == goal_seq_) {
          navigate_to_pose_result_callback(result);
        }
      };

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

    // 新しいゴールでは速度上限を解除し、最後のwaypointに近づいたらフィードバックで上限をかける
    final_approach_slowdown_active_ = false;
    publish_speed_limit(0.0);
  }

  void navigate_to_pose_feedback_callback(const std::shared_ptr<const NavigateToPose::Feedback> feedback)
  {
    if (is_waiting_for_input_ || has_resend_waypoint_ ||
      current_group_index_ >= waypoint_groups_.size() ||
      current_waypoint_index_ >= waypoint_groups_[current_group_index_].waypoints.size())
    {
      return;
    }
    const WaypointGroup & current_group = waypoint_groups_[current_group_index_];
    const Waypoint & wp = current_group.waypoints[current_waypoint_index_];
    const double dist = std::hypot(
      feedback->current_pose.pose.position.x - wp.x,
      feedback->current_pose.pose.position.y - wp.y);

    // 通過地点: 手前まで来たら現在のゴールを完了させずに次のゴールで上書きする
    if (enable_pass_through_ && is_pass_through_waypoint()) {
      if (dist <= pass_through_distance_) {
        RCLCPP_INFO(
          this->get_logger(),
          "Within %.2f m of waypoint %zu in group '%s'; passing through to the next waypoint.",
          dist, current_waypoint_index_ + 1, current_group.group_name.c_str());
        current_waypoint_index_++;
        send_next_goal();
      }
      return;
    }

    // 停止地点: 近づいたら速度を落とす
    if (final_approach_slowdown_active_ || final_approach_speed_ <= 0.0 || !is_final_waypoint()) {
      return;
    }
    if (dist > final_approach_distance_) {
      return;
    }
    final_approach_slowdown_active_ = true;
    RCLCPP_INFO(
      this->get_logger(), "Within %.2f m of the final waypoint; slowing down.", dist);
    publish_speed_limit(final_approach_speed_);
  }

  void navigate_to_pose_result_callback(const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult & result)
  {
    publish_follow_mode(false);

    const ResendDecision decision = compute_resend_decision();
    if (decision != ResendDecision::None) {
      WaypointGroup & current_group = waypoint_groups_[current_group_index_];
      Waypoint wp = current_group.waypoints[current_waypoint_index_];
      // 追従モードは使わないため、再送も通常のナビゲーションで行う
      const bool enable_follow_mode = false;
      RCLCPP_WARN(
        this->get_logger(),
        "Robot is still away from the last waypoint of group '%s'. "
        "Decision: %s. Retrying the same waypoint in 5 seconds.",
        current_group.group_name.c_str(),
        decision == ResendDecision::ResendWithFollow ? "resend with follow mode"
                                                     : "resend without follow mode");
      // Cancel any existing timer and schedule a resend.
      resend_timer_.reset();
      resend_waypoint_ = wp;
      resend_follow_mode_ = enable_follow_mode;
      has_resend_waypoint_ = true;
      resend_timer_ = this->create_wall_timer(
        5s, [this]() {
          resend_timer_.reset();
          if (has_resend_waypoint_) {
            send_navigate_to_pose(resend_waypoint_, resend_follow_mode_);
            has_resend_waypoint_ = false;
          }
        });
      return;
    }

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

  void proceed_callback(const std_msgs::msg::Empty::SharedPtr msg)
  {
    (void)msg;
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

  ResendDecision compute_resend_decision()
  {
    if (waypoint_groups_.empty()) {
      return ResendDecision::None;
    }
    WaypointGroup & current_group = waypoint_groups_[current_group_index_];
    if (!current_group.require_input) {
      return ResendDecision::None;
    }
    if (current_waypoint_index_ >= current_group.waypoints.size()) {
      return ResendDecision::None;
    }
    const bool is_last_in_group = current_waypoint_index_ == current_group.waypoints.size() - 1;
    if (!is_last_in_group) {
      return ResendDecision::None;
    }

    try {
      geometry_msgs::msg::TransformStamped tf =
        tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
      const Waypoint & wp = current_group.waypoints[current_waypoint_index_];
      const double dx = tf.transform.translation.x - wp.x;
      const double dy = tf.transform.translation.y - wp.y;
      const double dist = std::hypot(dx, dy);

      if (dist > 2.0) {
        RCLCPP_WARN(
          this->get_logger(),
          "Distance to last waypoint is %.2f m (> 2.0 m); will resend with follow mode.",
          dist);
        return ResendDecision::ResendWithFollow;
      }
      if (dist >= 1.0) {
        RCLCPP_WARN(
          this->get_logger(),
          "Distance to last waypoint is %.2f m (>= 1.0 m and <= 2.0 m); will resend with follow mode disabled.",
          dist);
        return ResendDecision::ResendWithoutFollow;
      }
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "TF lookup failed when checking waypoint distance: %s", ex.what());
    }

    return ResendDecision::None;
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
