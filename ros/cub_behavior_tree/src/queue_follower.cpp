// 行列区間を走るノード。
//
// QueueFollowアクションで start -> end の直線(map座標)を受け取り、
//   1. 直線に沿ってpure pursuitで走る
//   2. 直線を中心とした幅 2*lane_half_width の通路の中で、一番近い前方の点(=前のロボット)との距離を見て
//      近ければ止まり、空いたら再発進する
//   3. end に着いたら止まってアクション完了
// を行う。通路は地図に固定しているので、機体が旋回しても通路の外の壁は無視される。
//
// 出力はNav2のcontroller_serverと同じ cmd_vel_nav なので、velocity_smoother と collision_monitor を通る。
// 行列走行中はwaypoint_navigatorがNav2のゴールをキャンセルしているので、出力がぶつかることはない。

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "cub_behavior_tree/action/queue_follow.hpp"

using QueueFollow = cub_behavior_tree::action::QueueFollow;
using GoalHandle = rclcpp_action::ServerGoalHandle<QueueFollow>;
using namespace std::chrono_literals;

class QueueFollower : public rclcpp::Node
{
public:
  QueueFollower()
  : Node("queue_follower")
  {
    // 通路の半幅[m]。直線からこれ以上横にある点は無視する
    lane_half_width_ = declare_parameter("lane_half_width", 0.4);
    // base_linkから機体前端までの距離[m]
    robot_front_offset_ = declare_parameter("robot_front_offset", 0.2);
    // 前方を見る距離[m]
    detect_range_ = declare_parameter("detect_range", 5.0);
    // 前の物体との空き距離がこれ以下なら止まる[m]
    stop_gap_ = declare_parameter("stop_gap", 0.6);
    // 止まっている時、空き距離がこれ以上になったら再発進する[m]
    resume_gap_ = declare_parameter("resume_gap", 1.0);
    // 速度[m/s]
    max_speed_ = declare_parameter("max_speed", 0.5);
    min_speed_ = declare_parameter("min_speed", 0.1);
    // 前の物体・endに近づいたときの減速の強さ (速度 = gain * 残り距離)
    slowdown_gain_ = declare_parameter("slowdown_gain", 0.5);
    // pure pursuitの先読み距離[m]
    lookahead_distance_ = declare_parameter("lookahead_distance", 1.0);
    max_angular_speed_ = declare_parameter("max_angular_speed", 0.6);
    // endまでの残りがこれ以下になったら到着[m]
    goal_tolerance_ = declare_parameter("goal_tolerance", 0.15);
    auto scan_topics = declare_parameter(
      "scan_topics", std::vector<std::string>{"/scan", "/scan_livox"});

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel_nav", 10);

    // scanは各トピックの最新1つだけ保持する
    scans_.resize(scan_topics.size());
    scan_times_.resize(scan_topics.size());
    for (size_t i = 0; i < scan_topics.size(); ++i) {
      scan_subs_.push_back(create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topics[i], rclcpp::SensorDataQoS(),
        [this, i](sensor_msgs::msg::LaserScan::SharedPtr msg) {
          scans_[i] = msg;
          scan_times_[i] = now();
        }));
    }

    action_server_ = rclcpp_action::create_server<QueueFollow>(
      this, "queue_follow",
      [](const rclcpp_action::GoalUUID &, std::shared_ptr<const QueueFollow::Goal>) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](std::shared_ptr<GoalHandle>) {return rclcpp_action::CancelResponse::ACCEPT;},
      [this](std::shared_ptr<GoalHandle> goal_handle) {on_new_goal(goal_handle);});

    timer_ = create_wall_timer(50ms, [this]() {control_loop();});  // 20Hz
  }

private:
  void on_new_goal(std::shared_ptr<GoalHandle> goal_handle)
  {
    if (goal_) {
      finish(false, "preempted by a new goal");
    }
    goal_ = goal_handle;
    waiting_ = false;
    RCLCPP_INFO(get_logger(), "Queue follow started");
  }

  void control_loop()
  {
    if (!goal_) {
      return;
    }
    if (goal_->is_canceling()) {
      stop();
      auto result = std::make_shared<QueueFollow::Result>();
      result->message = "canceled";
      goal_->canceled(result);
      goal_.reset();
      return;
    }

    // 直線の情報: 始点a, 単位方向ベクトル(ux, uy), 長さ
    const auto & a = goal_->get_goal()->start;
    const auto & b = goal_->get_goal()->end;
    const double length = std::hypot(b.x - a.x, b.y - a.y);
    if (length < 0.1) {
      finish(false, "start and end are too close");
      return;
    }
    const double ux = (b.x - a.x) / length;
    const double uy = (b.y - a.y) / length;

    // ロボットの位置
    geometry_msgs::msg::TransformStamped robot_tf;
    try {
      robot_tf = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "TF error: %s", e.what());
      stop();
      return;
    }
    const double rx = robot_tf.transform.translation.x;
    const double ry = robot_tf.transform.translation.y;
    const double ryaw = tf2::getYaw(robot_tf.transform.rotation);

    // 直線に沿ったロボットの位置 s (始点からの距離)
    const double s_robot = (rx - a.x) * ux + (ry - a.y) * uy;
    const double remaining = length - s_robot;
    if (remaining <= goal_tolerance_) {
      finish(true, "reached the end of the queue");
      return;
    }

    // 前の物体までの空き距離 (無ければ負)
    const double gap = find_front_gap(a, ux, uy, s_robot);

    // 停止・再発進の判定 (停止と再発進で閾値を変えて、チラつかないようにしている)
    if (!waiting_ && gap >= 0.0 && gap <= stop_gap_) {
      waiting_ = true;
      RCLCPP_INFO(get_logger(), "Waiting: gap %.2f m", gap);
    } else if (waiting_ && (gap < 0.0 || gap >= resume_gap_)) {
      waiting_ = false;
      RCLCPP_INFO(get_logger(), "Resume");
    }

    // 速度: 前の物体・endに近づくほど遅くする
    double v = 0.0;
    if (!waiting_) {
      v = std::min(max_speed_, slowdown_gain_ * remaining);
      if (gap >= 0.0) {
        v = std::min(v, slowdown_gain_ * (gap - stop_gap_));
      }
      v = std::max(v, min_speed_);
    }

    // 旋回: pure pursuit。直線上で lookahead_distance 先の点を目指す
    const double tx = a.x + ux * (s_robot + lookahead_distance_);
    const double ty = a.y + uy * (s_robot + lookahead_distance_);
    // 目標点をロボット座標に変換
    const double dx = tx - rx;
    const double dy = ty - ry;
    const double local_x = std::cos(ryaw) * dx + std::sin(ryaw) * dy;
    const double local_y = -std::sin(ryaw) * dx + std::cos(ryaw) * dy;
    const double curvature = 2.0 * local_y / (local_x * local_x + local_y * local_y);
    const double w = std::clamp(v * curvature, -max_angular_speed_, max_angular_speed_);

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = v;
    cmd.angular.z = w;
    cmd_pub_->publish(cmd);

    auto feedback = std::make_shared<QueueFollow::Feedback>();
    feedback->distance_to_end = remaining;
    feedback->gap = gap;
    feedback->waiting = waiting_;
    goal_->publish_feedback(feedback);
  }

  // 通路の中にあるscan点のうち、一番近い前方の点までの空き距離(機体前端から、直線に沿って)を返す。
  // 見つからなければ -1。
  // 改良案: ノイズ対策で「n点以上まとまっているもの」だけを物体とみなす、など。
  double find_front_gap(
    const geometry_msgs::msg::Point & a, double ux, double uy, double s_robot)
  {
    const double s_front = s_robot + robot_front_offset_;
    double min_gap = -1.0;

    for (size_t k = 0; k < scans_.size(); ++k) {
      const auto & scan = scans_[k];
      // 0.5秒以上届いていないscanは使わない
      if (!scan || (now() - scan_times_[k]).seconds() > 0.5) {
        continue;
      }
      geometry_msgs::msg::TransformStamped scan_tf;
      try {
        scan_tf = tf_buffer_->lookupTransform("map", scan->header.frame_id, tf2::TimePointZero);
      } catch (const tf2::TransformException &) {
        continue;
      }
      tf2::Transform scan_to_map;
      tf2::fromMsg(scan_tf.transform, scan_to_map);

      for (size_t i = 0; i < scan->ranges.size(); ++i) {
        const double r = scan->ranges[i];
        if (!std::isfinite(r) || r < scan->range_min || r > scan->range_max) {
          continue;
        }
        // scan点をmap座標へ
        const double angle = scan->angle_min + i * scan->angle_increment;
        const tf2::Vector3 p = scan_to_map * tf2::Vector3(r * std::cos(angle), r * std::sin(angle), 0.0);

        // 直線に沿った位置 s と 横ずれ d
        const double s = (p.x() - a.x) * ux + (p.y() - a.y) * uy;
        const double d = -(p.x() - a.x) * uy + (p.y() - a.y) * ux;
        if (std::fabs(d) > lane_half_width_) {
          continue;  // 通路の外 (壁など)
        }
        const double gap = s - s_front;
        if (gap < 0.0 || gap > detect_range_) {
          continue;  // 機体より後ろ、または遠すぎる
        }
        if (min_gap < 0.0 || gap < min_gap) {
          min_gap = gap;
        }
      }
    }
    return min_gap;
  }

  void finish(bool success, const std::string & message)
  {
    stop();
    auto result = std::make_shared<QueueFollow::Result>();
    result->success = success;
    result->message = message;
    if (success) {
      goal_->succeed(result);
    } else {
      goal_->abort(result);
    }
    goal_.reset();
    RCLCPP_INFO(get_logger(), "Queue follow finished: %s", message.c_str());
  }

  void stop()
  {
    cmd_pub_->publish(geometry_msgs::msg::Twist());
  }

  double lane_half_width_, robot_front_offset_, detect_range_;
  double stop_gap_, resume_gap_;
  double max_speed_, min_speed_, slowdown_gain_;
  double lookahead_distance_, max_angular_speed_, goal_tolerance_;

  std::shared_ptr<GoalHandle> goal_;  // 実行中のゴール (なければnull)
  bool waiting_{false};               // 前の物体待ちで停止中

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr> scan_subs_;
  std::vector<sensor_msgs::msg::LaserScan::SharedPtr> scans_;
  std::vector<rclcpp::Time> scan_times_;  // 各scanの受信時刻
  rclcpp_action::Server<QueueFollow>::SharedPtr action_server_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<QueueFollower>());
  rclcpp::shutdown();
  return 0;
}
