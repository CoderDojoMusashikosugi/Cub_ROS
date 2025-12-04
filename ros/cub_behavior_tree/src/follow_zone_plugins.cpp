#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <atomic>
#include <string>
#include <thread>
#include <vector>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/utils.h>
#include <tf2/time.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <std_msgs/msg/bool.hpp>

#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <nav2_behavior_tree/bt_action_node.hpp>

#include "cub_behavior_tree/action/acquire_front_target.hpp"

namespace cub_behavior_tree
{

class FollowModeEnabled : public BT::ConditionNode
{
public:
  FollowModeEnabled(const std::string & name, const BT::NodeConfiguration & conf)
  : BT::ConditionNode(name, conf)
  {
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("topic", std::string("/follow_mode_enabled"), "Topic that enables front-follow mode")
    };
  }

  BT::NodeStatus tick() override
  {
    ensure_subscription();

    // Pump callbacks in case the executor thread is starved; ensures the latched
    // follow_mode flag is actually received before evaluating the condition.
    rclcpp::spin_some(node_);

    if (!got_message_.load()) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 2000,
        "FollowModeEnabled: waiting for follow mode flag on topic %s", topic_.c_str());
      return BT::NodeStatus::FAILURE;
    }

    return enabled_.load() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

private:
  void ensure_subscription()
  {
    std::lock_guard<std::mutex> lock(sub_mutex_);
    if (sub_) {
      return;
    }

    topic_ = "/follow_mode_enabled";
    (void)getInput("topic", topic_);

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    sub_ = node_->create_subscription<std_msgs::msg::Bool>(
      topic_, qos,
      [this](std_msgs::msg::Bool::SharedPtr msg)
      {
        const bool previous = enabled_.exchange(msg->data);
        const bool first = !got_message_.exchange(true);
        if (first || previous != msg->data) {
          RCLCPP_INFO(
            node_->get_logger(), "FollowModeEnabled: received %s on %s",
            msg->data ? "true" : "false", topic_.c_str());
        }
      });

    RCLCPP_INFO(node_->get_logger(), "FollowModeEnabled: subscribed to %s", topic_.c_str());
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_;
  std::atomic_bool enabled_{false};
  std::atomic_bool got_message_{false};
  std::string topic_;
  std::mutex sub_mutex_;
};

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
    
    RCLCPP_INFO(node_->get_logger(), "IsFollowZone radius_sq = %f, dist_sq = %f", radius_sq, dist_sq);

    return dist_sq <= radius_sq ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
};

class SetFrontFollowServerState : public BT::SyncActionNode
{
public:
  SetFrontFollowServerState(const std::string & name, const BT::NodeConfiguration & conf)
  : BT::SyncActionNode(name, conf)
  {
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<bool>("activate", true, "True to activate, false to deactivate")
    };
  }

  BT::NodeStatus tick() override
  {
    bool activate = true;
    (void)getInput("activate", activate);

    const bool previous = active_.exchange(activate);
    if (activate && !previous) {
      RCLCPP_INFO(node_->get_logger(), "Front-follow server activated");
    } else if (!activate && previous) {
      RCLCPP_INFO(node_->get_logger(), "Front-follow server deactivated");
    }

    return BT::NodeStatus::SUCCESS;
  }

  static bool is_active()
  {
    return active_.load();
  }

private:
  static std::atomic_bool active_;
  rclcpp::Node::SharedPtr node_;
};

std::atomic_bool SetFrontFollowServerState::active_{false};

class AcquireFrontTargetAction : public nav2_behavior_tree::BtActionNode<cub_behavior_tree::action::AcquireFrontTarget>
{
public:
  AcquireFrontTargetAction(const std::string & name, const BT::NodeConfiguration & conf)
  : nav2_behavior_tree::BtActionNode<cub_behavior_tree::action::AcquireFrontTarget>(
      name, "acquire_front_target", conf)
  {
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("goal", "Navigation goal pose"),
      BT::InputPort<std::string>("scan_topic", std::string("/scan"), "LaserScan topic"),
      BT::InputPort<double>("max_offset_deg", 25.0, "Allowed heading offset (deg) from goal direction"),
      BT::InputPort<double>("front_fov_deg", 30.0, "Field of view to consider around robot front (deg)"),
      BT::InputPort<double>("max_range", 3.0, "Max range to consider (m)"),
      BT::InputPort<std::string>("robot_frame", std::string("base_link"), "Robot base TF frame"),
      BT::InputPort<double>("standoff_distance", 0.5, "Distance to stay in front of the detected object (m)"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("target", "Selected target pose in map frame")
    };
  }

  void on_tick() override
  {
    geometry_msgs::msg::PoseStamped goal;
    if (!getInput("goal", goal)) {
      RCLCPP_WARN(node_->get_logger(), "AcquireFrontTarget: missing goal input");
      return;
    }

    goal_.goal = goal;
    (void)getInput("max_offset_deg", goal_.max_offset_deg);
    (void)getInput("front_fov_deg", goal_.front_fov_deg);
    (void)getInput("max_range", goal_.max_range);
    (void)getInput("robot_frame", goal_.robot_frame);
    (void)getInput("standoff_distance", goal_.standoff_distance);
    (void)getInput("scan_topic", goal_.scan_topic);
  }

  BT::NodeStatus on_success() override
  {
    setOutput("target", result_.result->target);
    return BT::NodeStatus::SUCCESS;
  }

private:
  rclcpp::Node::SharedPtr node_;
};

}  // namespace cub_behavior_tree

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<cub_behavior_tree::FollowModeEnabled>("FollowModeEnabled");
  factory.registerNodeType<cub_behavior_tree::IsInFollowZone>("IsInFollowZone");
  factory.registerNodeType<cub_behavior_tree::AcquireFrontTargetAction>("AcquireFrontTarget");
  factory.registerNodeType<cub_behavior_tree::SetFrontFollowServerState>("SetFrontFollowServerState");
}
