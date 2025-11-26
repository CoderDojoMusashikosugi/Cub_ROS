#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2/time.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/bt_factory.h>

namespace cub_behavior_tree
{

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

class IsInFollowZone : public BT::ConditionNode
{
public:
  IsInFollowZone(const std::string & name, const BT::NodeConfiguration & conf)
  : BT::ConditionNode(name, conf)
  {
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    tf_buffer_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("goal", "Navigation goal pose"),
      BT::InputPort<double>("center_x", "Override zone center X (map frame)"),
      BT::InputPort<double>("center_y", "Override zone center Y (map frame)"),
      BT::InputPort<double>("radius", 1.0, "Radius of follow zone (m)"),
      BT::InputPort<std::string>("robot_frame", std::string("base_link"), "Robot base TF frame")
    };
  }

  BT::NodeStatus tick() override
  {
    geometry_msgs::msg::PoseStamped goal;
    if (!getInput("goal", goal)) {
      RCLCPP_WARN(node_->get_logger(), "IsInFollowZone: missing goal input");
      return BT::NodeStatus::FAILURE;
    }

    double center_x = goal.pose.position.x;
    double center_y = goal.pose.position.y;
    (void)getInput("center_x", center_x);
    (void)getInput("center_y", center_y);

    double radius = 1.0;
    (void)getInput("radius", radius);
    if (radius <= 0.0) {
      RCLCPP_WARN(node_->get_logger(), "IsInFollowZone: radius must be > 0");
      return BT::NodeStatus::FAILURE;
    }

    std::string robot_frame = "base_link";
    (void)getInput("robot_frame", robot_frame);

    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform("map", robot_frame, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(node_->get_logger(), "IsInFollowZone: TF lookup failed: %s", ex.what());
      return BT::NodeStatus::FAILURE;
    }

    const double dx = tf.transform.translation.x - center_x;
    const double dy = tf.transform.translation.y - center_y;
    const double dist_sq = dx * dx + dy * dy;
    const double radius_sq = radius * radius;

    return dist_sq <= radius_sq ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
};

class AcquireFrontTarget : public BT::SyncActionNode
{
public:
  AcquireFrontTarget(const std::string & name, const BT::NodeConfiguration & conf)
  : BT::SyncActionNode(name, conf)
  {
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    tf_buffer_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("goal", "Navigation goal pose"),
      BT::InputPort<std::string>("scan_topic", std::string("/scan"), "LaserScan topic"),
      BT::InputPort<double>("max_offset_deg", 25.0, "Allowed heading offset (deg) from goal direction"),
      BT::InputPort<double>("max_range", 3.0, "Max range to consider (m)"),
      BT::InputPort<std::string>("robot_frame", std::string("base_link"), "Robot base TF frame"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("target", "Selected target pose in map frame")
    };
  }

  BT::NodeStatus tick() override
  {
    ensure_subscription();

    geometry_msgs::msg::PoseStamped goal;
    if (!getInput("goal", goal)) {
      RCLCPP_WARN(node_->get_logger(), "AcquireFrontTarget: missing goal input");
      return BT::NodeStatus::FAILURE;
    }

    double max_offset_deg = 25.0;
    (void)getInput("max_offset_deg", max_offset_deg);
    const double max_offset_rad = max_offset_deg * M_PI / 180.0;

    double max_range = 3.0;
    (void)getInput("max_range", max_range);

    std::string robot_frame = "base_link";
    (void)getInput("robot_frame", robot_frame);

    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform("map", robot_frame, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(node_->get_logger(), "AcquireFrontTarget: TF lookup failed: %s", ex.what());
      return BT::NodeStatus::FAILURE;
    }

    geometry_msgs::msg::PoseArray::SharedPtr objects_copy;
    sensor_msgs::msg::LaserScan::SharedPtr scan_copy;
    {
      std::lock_guard<std::mutex> lock(objects_mutex_);
      scan_copy = last_scan_;
    }

    if (!scan_copy) {
      RCLCPP_DEBUG(node_->get_logger(), "AcquireFrontTarget: no scan received");
      return BT::NodeStatus::FAILURE;
    }

    const double goal_yaw = yaw_from_quat(goal.pose.orientation);
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
      const double offset = normalize_angle(angle - goal_yaw_base);
      if (std::fabs(offset) > max_offset_rad) {
        continue;
      }

      geometry_msgs::msg::PointStamped p_base;
      p_base.header.frame_id = scan_copy->header.frame_id;
      p_base.header.stamp = node_->get_clock()->now();
      p_base.point.x = r * std::cos(angle);
      p_base.point.y = r * std::sin(angle);
      p_base.point.z = 0.0;

      geometry_msgs::msg::PointStamped p_map;
      try {
        tf_buffer_->transform(p_base, p_map, "map", tf2::durationFromSec(0.05));
      } catch (const tf2::TransformException & ex) {
        RCLCPP_DEBUG(node_->get_logger(), "AcquireFrontTarget: transform failed: %s", ex.what());
        continue;
      }

      if (r < closest_dist) {
        closest_dist = r;
        best_target.header.frame_id = "map";
        best_target.header.stamp = node_->get_clock()->now();
        best_target.pose.position.x = p_map.point.x;
        best_target.pose.position.y = p_map.point.y;
        best_target.pose.position.z = 0.0;
        best_target.pose.orientation = goal.pose.orientation;  // align with goal heading
      }
    }

    if (closest_dist == std::numeric_limits<double>::max()) {
      RCLCPP_DEBUG(node_->get_logger(), "AcquireFrontTarget: no target within constraints");
      return BT::NodeStatus::FAILURE;
    }

    setOutput("target", best_target);
    return BT::NodeStatus::SUCCESS;
  }

private:
  void ensure_subscription()
  {
    if (scan_sub_) {
      return;
    }

    std::string topic = "/scan";
    (void)getInput("scan_topic", topic);

    scan_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
      topic, rclcpp::QoS(10),
      [this](sensor_msgs::msg::LaserScan::SharedPtr msg)
      {
        std::lock_guard<std::mutex> lock(objects_mutex_);
        last_scan_ = std::move(msg);
      });
  }

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  sensor_msgs::msg::LaserScan::SharedPtr last_scan_;
  std::mutex objects_mutex_;
};

}  // namespace cub_behavior_tree

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<cub_behavior_tree::IsInFollowZone>("IsInFollowZone");
  factory.registerNodeType<cub_behavior_tree::AcquireFrontTarget>("AcquireFrontTarget");
}
