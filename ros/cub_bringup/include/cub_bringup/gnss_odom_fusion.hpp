#ifndef CUB_BRINGUP__GNSS_ODOM_FUSION_HPP_
#define CUB_BRINGUP__GNSS_ODOM_FUSION_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>

namespace cub_bringup
{

class GnssOdomFusion : public rclcpp::Node
{
public:
  GnssOdomFusion();
  ~GnssOdomFusion();

private:
  void gnss_callback(const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg);
  void gps_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg);
  void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void initialpose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg);
  void timer_callback();
  void publish_tf();
  void update_yaw_from_odom(const nav_msgs::msg::Odometry & odom);
  void update_yaw_from_gnss_movement(const nav_msgs::msg::Odometry & current_odom);
  bool is_gnss_valid() const;

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr gps_pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_sub_;

  // Publisher
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;

  // TF Broadcaster
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // State variables
  double gnss_x_;
  double gnss_y_;
  double gnss_z_;
  bool gnss_received_;
  bool gnss_valid_;
  rclcpp::Time last_gnss_time_;

  // Previous GNSS position for movement calculation
  double last_gnss_x_;
  double last_gnss_y_;
  bool last_gnss_position_valid_;

  // GNSS position at last yaw correction
  double gnss_x_at_last_correction_;
  double gnss_y_at_last_correction_;

  // Odometry position at last yaw correction
  double odom_x_at_last_correction_;
  double odom_y_at_last_correction_;
  bool odom_correction_reference_valid_;

  double current_yaw_;
  double last_odom_yaw_;
  bool odom_initialized_;

  rclcpp::Time last_odom_time_;

  // Parameters
  double gnss_timeout_;
  bool use_gnss_altitude_;
  double tf_publish_rate_;
  double initial_yaw_;
  bool use_gnss_yaw_correction_;
  double min_gnss_movement_for_yaw_;
  double min_odom_movement_for_yaw_;
  double max_distance_mismatch_ratio_;
  double gnss_yaw_weight_;

  // Latest odometry message for distance calculation
  nav_msgs::msg::Odometry latest_odom_;
};

}  // namespace cub_bringup

#endif  // CUB_BRINGUP__GNSS_ODOM_FUSION_HPP_
