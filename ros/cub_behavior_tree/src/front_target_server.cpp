#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include "cub_behavior_tree/action/acquire_front_target.hpp"
#include "cub_behavior_tree/front_target_server.hpp"

namespace cub_behavior_tree
{

using AcquireFrontTarget = cub_behavior_tree::action::AcquireFrontTarget;
using GoalHandle = rclcpp_action::ServerGoalHandle<AcquireFrontTarget>;

inline double normalize_angle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

inline double yaw_from_quat(const geometry_msgs::msg::Quaternion & q)
{
  tf2::Quaternion tf_q;
  tf2::fromMsg(q, tf_q);
  return tf2::getYaw(tf_q);
}

inline geometry_msgs::msg::Quaternion quat_from_yaw(double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  return tf2::toMsg(q);
}

class FrontTargetServer : public rclcpp::Node
{
public:
  explicit FrontTargetServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("front_target_server", options)
  {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    action_server_ = rclcpp_action::create_server<AcquireFrontTarget>(
      this,
      "acquire_front_target",
      std::bind(&FrontTargetServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&FrontTargetServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&FrontTargetServer::handle_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "front_target_server ready on action 'acquire_front_target'");
  }

private:
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const AcquireFrontTarget::Goal>)
  {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle>)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
  {
    std::thread{std::bind(&FrontTargetServer::execute, this, std::placeholders::_1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandle> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<AcquireFrontTarget::Result>();

    ensure_scan_subscription(goal->scan_topic.empty() ? "/scan" : goal->scan_topic);

    sensor_msgs::msg::LaserScan::SharedPtr scan_copy;
    {
      std::lock_guard<std::mutex> lock(scan_mutex_);
      scan_copy = last_scan_;
    }

    if (!scan_copy) {
      RCLCPP_WARN(get_logger(), "No scan available; falling back to navigation goal");
      result->target = goal->goal;
      result->fallback_to_goal = true;
      goal_handle->succeed(result);
      return;
    }

    std::string robot_frame = goal->robot_frame.empty() ? "base_link" : goal->robot_frame;

    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform("map", robot_frame, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(get_logger(), "TF lookup failed: %s", ex.what());
      result->target = goal->goal;
      result->fallback_to_goal = true;
      goal_handle->succeed(result);
      return;
    }

    const double max_offset_deg = goal->max_offset_deg;
    const double max_offset_rad = max_offset_deg * M_PI / 180.0;
    const double front_fov_deg = goal->front_fov_deg > 0.0 ? goal->front_fov_deg : 30.0;
    const double half_front_rad = (front_fov_deg * M_PI / 180.0) * 0.5;
    double max_range = goal->max_range;
    if (max_range > 3.0) {
      RCLCPP_WARN(get_logger(), "max_range limited to 3.0 m");
      max_range = 3.0;
    }

    double standoff_distance = goal->standoff_distance;
    if (standoff_distance < 0.0) {
      standoff_distance = 0.0;
    }

    // Lookup transform between scan frame and base
    geometry_msgs::msg::TransformStamped scan_to_base;
    try {
      scan_to_base = tf_buffer_->lookupTransform(
        robot_frame, scan_copy->header.frame_id, scan_copy->header.stamp);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(get_logger(), "scan->base TF lookup failed: %s", ex.what());
      result->target = goal->goal;
      result->fallback_to_goal = true;
      goal_handle->succeed(result);
      return;
    }

    const double goal_yaw = yaw_from_quat(goal->goal.pose.orientation);
    const double robot_yaw = yaw_from_quat(tf.transform.rotation);
    const double goal_yaw_base = normalize_angle(goal_yaw - robot_yaw);

    double closest_dist = std::numeric_limits<double>::max();
    geometry_msgs::msg::PoseStamped best_target;

    for (size_t i = 0; i < scan_copy->ranges.size(); ++i) {
      const double r = scan_copy->ranges[i];
      if (std::isnan(r) || std::isinf(r) || r <= 0.0 || r > max_range) {
        continue;
      }

      const double angle = scan_copy->angle_min + static_cast<double>(i) * scan_copy->angle_increment;
      geometry_msgs::msg::PointStamped p_scan;
      p_scan.header.frame_id = scan_copy->header.frame_id;
      p_scan.header.stamp = scan_copy->header.stamp;
      p_scan.point.x = r * std::cos(angle);
      p_scan.point.y = r * std::sin(angle);
      p_scan.point.z = 0.0;

      geometry_msgs::msg::PointStamped p_base;
      try {
        tf2::doTransform(p_scan, p_base, scan_to_base);
      } catch (const tf2::TransformException & ex) {
        RCLCPP_DEBUG(get_logger(), "scan->base transform failed: %s", ex.what());
        continue;
      }

      const double angle_base = std::atan2(p_base.point.y, p_base.point.x);
      if (std::fabs(angle_base) > half_front_rad) {
        continue;
      }

      const double offset = normalize_angle(angle_base - goal_yaw_base);
      if (std::fabs(offset) > max_offset_rad) {
        continue;
      }

      const double dist_base = std::hypot(p_base.point.x, p_base.point.y);
      if (dist_base > max_range) {
        continue;
      }

      geometry_msgs::msg::PointStamped p_map;
      try {
        tf_buffer_->transform(p_base, p_map, "map", tf2::durationFromSec(0.05));
      } catch (const tf2::TransformException & ex) {
        RCLCPP_DEBUG(get_logger(), "transform failed: %s", ex.what());
        continue;
      }

      if (dist_base < closest_dist) {
        closest_dist = dist_base;
        best_target.header.frame_id = "map";
        best_target.header.stamp = now();
        best_target.pose.position.x = p_map.point.x;
        best_target.pose.position.y = p_map.point.y;
        best_target.pose.position.z = 0.0;
        best_target.pose.orientation = goal->goal.pose.orientation;
      }
    }

    if (closest_dist == std::numeric_limits<double>::max()) {
      RCLCPP_INFO(get_logger(), "No front target found; using navigation goal");
      result->target = goal->goal;
      result->fallback_to_goal = true;
      goal_handle->succeed(result);
      return;
    }

    const double robot_x = tf.transform.translation.x;
    const double robot_y = tf.transform.translation.y;
    const double dx = best_target.pose.position.x - robot_x;
    const double dy = best_target.pose.position.y - robot_y;
    const double dist_to_target = std::hypot(dx, dy);

    if (dist_to_target > 1e-3 && standoff_distance > 0.0) {
      const double stop_dist = std::max(0.05, dist_to_target - standoff_distance);
      const double scale = stop_dist / dist_to_target;
      best_target.pose.position.x = robot_x + dx * scale;
      best_target.pose.position.y = robot_y + dy * scale;
    }

    const double yaw_to_target = std::atan2(
      best_target.pose.position.y - robot_y, best_target.pose.position.x - robot_x);
    best_target.pose.orientation = quat_from_yaw(yaw_to_target);

    result->target = best_target;
    result->fallback_to_goal = false;
    goal_handle->succeed(result);
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp_action::Server<AcquireFrontTarget>::SharedPtr action_server_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  sensor_msgs::msg::LaserScan::SharedPtr last_scan_;
  std::mutex scan_mutex_;
  std::mutex sub_mutex_;
  std::string scan_topic_ {"/scan"};

  void ensure_scan_subscription(const std::string & topic)
  {
    std::lock_guard<std::mutex> lock(sub_mutex_);
    if (scan_sub_ && topic == scan_topic_) {
      return;
    }

    scan_topic_ = topic;
    // Use SensorData QoS for low-latency scan handling (best effort, small queue)
    auto qos = rclcpp::SensorDataQoS();
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, qos,
      [this](sensor_msgs::msg::LaserScan::SharedPtr msg)
      {
        std::lock_guard<std::mutex> lock(scan_mutex_);
        last_scan_ = std::move(msg);
      });
    RCLCPP_INFO(get_logger(), "Subscribed to scan topic: %s", scan_topic_.c_str());
  }
};

std::shared_ptr<rclcpp::Node> create_front_target_server(const rclcpp::NodeOptions & options)
{
  return std::make_shared<FrontTargetServer>(options);
}

}  // namespace cub_behavior_tree

RCLCPP_COMPONENTS_REGISTER_NODE(cub_behavior_tree::FrontTargetServer)
