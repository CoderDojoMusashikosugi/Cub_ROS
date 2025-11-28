#include "cub_bringup/gnss_odom_fusion.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace cub_bringup
{

GnssOdomFusion::GnssOdomFusion()
: Node("gnss_odom_fusion"),
  gnss_x_(0.0),
  gnss_y_(0.0),
  gnss_z_(0.0),
  gnss_received_(false),
  gnss_valid_(false),
  last_gnss_x_(0.0),
  last_gnss_y_(0.0),
  last_gnss_position_valid_(false),
  gnss_x_at_last_correction_(0.0),
  gnss_y_at_last_correction_(0.0),
  odom_x_at_last_correction_(0.0),
  odom_y_at_last_correction_(0.0),
  odom_correction_reference_valid_(false),
  current_yaw_(0.0),
  last_odom_yaw_(0.0),
  odom_initialized_(false),
  gps_origin_lat_(0.0),
  gps_origin_lon_(0.0),
  gps_origin_alt_(0.0),
  use_manual_origin_(false)
{
  // Declare parameters
  this->declare_parameter<double>("gnss_timeout", 1.0);
  this->declare_parameter<bool>("use_gnss_altitude", false);
  this->declare_parameter<double>("tf_publish_rate", 50.0);
  this->declare_parameter<double>("initial_yaw", 0.0);
  this->declare_parameter<bool>("use_gnss_yaw_correction", true);
  this->declare_parameter<double>("min_gnss_movement_for_yaw", 2.0);
  this->declare_parameter<double>("min_odom_movement_for_yaw", 2.0);
  this->declare_parameter<double>("max_distance_mismatch_ratio", 0.3);
  this->declare_parameter<double>("gnss_yaw_weight", 0.3);

  // GNSS origin parameters
  this->declare_parameter<bool>("use_manual_origin", false);
  this->declare_parameter<double>("gps_origin_lat", 0.0);
  this->declare_parameter<double>("gps_origin_lon", 0.0);
  this->declare_parameter<double>("gps_origin_alt", 0.0);

  // Get parameters
  this->get_parameter("gnss_timeout", gnss_timeout_);
  this->get_parameter("use_gnss_altitude", use_gnss_altitude_);
  this->get_parameter("tf_publish_rate", tf_publish_rate_);
  this->get_parameter("initial_yaw", initial_yaw_);
  this->get_parameter("use_gnss_yaw_correction", use_gnss_yaw_correction_);
  this->get_parameter("min_gnss_movement_for_yaw", min_gnss_movement_for_yaw_);
  this->get_parameter("min_odom_movement_for_yaw", min_odom_movement_for_yaw_);
  this->get_parameter("max_distance_mismatch_ratio", max_distance_mismatch_ratio_);
  this->get_parameter("gnss_yaw_weight", gnss_yaw_weight_);

  this->get_parameter("use_manual_origin", use_manual_origin_);
  this->get_parameter("gps_origin_lat", gps_origin_lat_);
  this->get_parameter("gps_origin_lon", gps_origin_lon_);
  this->get_parameter("gps_origin_alt", gps_origin_alt_);

  current_yaw_ = initial_yaw_;
  last_gnss_time_ = this->now();
  last_odom_time_ = this->now();

  // Initialize subscribers
  gnss_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
    "/fix",
    rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::BestEffort),
    std::bind(&GnssOdomFusion::gnss_callback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom",
    rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::Reliable),
    std::bind(&GnssOdomFusion::odom_callback, this, std::placeholders::_1));

  initialpose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/initialpose",
    rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::Reliable),
    std::bind(&GnssOdomFusion::initialpose_callback, this, std::placeholders::_1));

  // Initialize publishers
  pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/gnss_pose",
    rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::Reliable));

  gps_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/gps_pose",
    rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::Reliable));

  // Initialize TF broadcaster
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // Setup timer for TF publishing
  auto timer_period = std::chrono::duration<double>(1.0 / tf_publish_rate_);
  timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(timer_period),
    std::bind(&GnssOdomFusion::timer_callback, this));

  RCLCPP_INFO(this->get_logger(), "GNSS-Odom Fusion Node initialized");
  RCLCPP_INFO(this->get_logger(), "  GNSS timeout: %.2f sec", gnss_timeout_);
  RCLCPP_INFO(this->get_logger(), "  Use GNSS altitude: %s", use_gnss_altitude_ ? "true" : "false");
  RCLCPP_INFO(this->get_logger(), "  TF publish rate: %.1f Hz", tf_publish_rate_);
  RCLCPP_INFO(this->get_logger(), "  Initial yaw: %.3f rad", initial_yaw_);
  RCLCPP_INFO(this->get_logger(), "  Use manual origin: %s", use_manual_origin_ ? "true" : "false");
  if (use_manual_origin_) {
    RCLCPP_INFO(this->get_logger(), "  GPS origin: lat=%.8f, lon=%.8f, alt=%.3f",
                gps_origin_lat_, gps_origin_lon_, gps_origin_alt_);
  }
  RCLCPP_INFO(this->get_logger(), "  Use GNSS yaw correction: %s", use_gnss_yaw_correction_ ? "true" : "false");
  if (use_gnss_yaw_correction_) {
    RCLCPP_INFO(this->get_logger(), "  Min GNSS movement for yaw: %.2f m", min_gnss_movement_for_yaw_);
    RCLCPP_INFO(this->get_logger(), "  Min Odom movement for yaw: %.2f m", min_odom_movement_for_yaw_);
    RCLCPP_INFO(this->get_logger(), "  Max distance mismatch ratio: %.2f", max_distance_mismatch_ratio_);
    RCLCPP_INFO(this->get_logger(), "  GNSS yaw weight: %.2f", gnss_yaw_weight_);
  }
}

GnssOdomFusion::~GnssOdomFusion()
{
}

void GnssOdomFusion::gnss_callback(const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg)
{
  // Check GNSS status: STATUS_SBAS_FIX (1) or STATUS_GBAS_FIX (2)
  if (msg->status.status == sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX ||
      msg->status.status == sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX)
  {
    gnss_valid_ = true;
    last_gnss_time_ = this->now();

    RCLCPP_DEBUG(this->get_logger(),
      "GNSS valid: status=%d, lat=%.8f, lon=%.8f",
      msg->status.status, msg->latitude, msg->longitude);

    // Convert GNSS to local pose
    convert_gnss_to_pose(msg);
  } else {
    gnss_valid_ = false;
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "GNSS status invalid: %d (required: STATUS_SBAS_FIX=1 or STATUS_GBAS_FIX=2)",
      msg->status.status);
  }
}

void GnssOdomFusion::convert_gnss_to_pose(const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg)
{
  // Initialize origin from first valid GNSS if not using manual origin
  static bool origin_initialized = false;
  if (!use_manual_origin_ && !origin_initialized) {
    gps_origin_lat_ = msg->latitude;
    gps_origin_lon_ = msg->longitude;
    gps_origin_alt_ = msg->altitude;
    origin_initialized = true;
    RCLCPP_INFO(this->get_logger(),
      "GPS origin auto-initialized: lat=%.8f, lon=%.8f, alt=%.3f",
      gps_origin_lat_, gps_origin_lon_, gps_origin_alt_);
  }

  // Convert lat/lon to local ENU coordinates
  // Using simple Equirectangular projection for short distances
  constexpr double EARTH_RADIUS = 6378137.0;  // WGS84 equatorial radius (m)

  double lat_rad = msg->latitude * M_PI / 180.0;
  double lon_rad = msg->longitude * M_PI / 180.0;
  double origin_lat_rad = gps_origin_lat_ * M_PI / 180.0;
  double origin_lon_rad = gps_origin_lon_ * M_PI / 180.0;

  // Calculate local coordinates (ENU frame)
  double x = EARTH_RADIUS * (lon_rad - origin_lon_rad) * std::cos(origin_lat_rad);
  double y = EARTH_RADIUS * (lat_rad - origin_lat_rad);
  double z = msg->altitude - gps_origin_alt_;

  // Store previous position before updating
  if (gnss_received_) {
    last_gnss_x_ = gnss_x_;
    last_gnss_y_ = gnss_y_;
    last_gnss_position_valid_ = true;
  }

  gnss_x_ = x;
  gnss_y_ = y;

  if (use_gnss_altitude_) {
    gnss_z_ = z;
  } else {
    gnss_z_ = 0.0;
  }

  gnss_received_ = true;

  // Publish GPS pose (before fusion)
  auto gps_pose_msg = geometry_msgs::msg::PoseWithCovarianceStamped();
  gps_pose_msg.header.stamp = msg->header.stamp;
  gps_pose_msg.header.frame_id = "map";
  gps_pose_msg.pose.pose.position.x = x;
  gps_pose_msg.pose.pose.position.y = y;
  gps_pose_msg.pose.pose.position.z = z;

  // Set orientation to identity (no orientation from GPS alone)
  gps_pose_msg.pose.pose.orientation.w = 1.0;
  gps_pose_msg.pose.pose.orientation.x = 0.0;
  gps_pose_msg.pose.pose.orientation.y = 0.0;
  gps_pose_msg.pose.pose.orientation.z = 0.0;

  // Copy covariance from NavSatFix to pose covariance (position only)
  // NavSatFix covariance: [lat, lon, alt]
  // PoseWithCovariance: 6x6 matrix [x, y, z, roll, pitch, yaw]
  for (int i = 0; i < 36; ++i) {
    gps_pose_msg.pose.covariance[i] = 0.0;
  }

  // Map lat->y, lon->x, alt->z covariances
  gps_pose_msg.pose.covariance[0] = msg->position_covariance[4];   // x variance (from lon)
  gps_pose_msg.pose.covariance[7] = msg->position_covariance[0];   // y variance (from lat)
  gps_pose_msg.pose.covariance[14] = msg->position_covariance[8];  // z variance (from alt)

  // Set high uncertainty for orientation (not measured by GPS)
  gps_pose_msg.pose.covariance[21] = 1000.0;  // roll variance
  gps_pose_msg.pose.covariance[28] = 1000.0;  // pitch variance
  gps_pose_msg.pose.covariance[35] = 1000.0;  // yaw variance

  gps_pose_pub_->publish(gps_pose_msg);

  // Update yaw from GNSS movement if enabled and we have valid odometry
  if (use_gnss_yaw_correction_ && odom_initialized_) {
    update_yaw_from_gnss_movement(latest_odom_);
  }

  RCLCPP_DEBUG(this->get_logger(),
    "GNSS converted to pose: x=%.3f, y=%.3f, z=%.3f",
    gnss_x_, gnss_y_, gnss_z_);
}

void GnssOdomFusion::odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  last_odom_time_ = this->now();
  latest_odom_ = *msg;  // Store latest odometry for distance calculation
  update_yaw_from_odom(*msg);
}

void GnssOdomFusion::update_yaw_from_odom(const nav_msgs::msg::Odometry & odom)
{
  // Extract yaw from odometry quaternion
  tf2::Quaternion q(
    odom.pose.pose.orientation.x,
    odom.pose.pose.orientation.y,
    odom.pose.pose.orientation.z,
    odom.pose.pose.orientation.w);

  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

  if (!odom_initialized_) {
    // First odometry message - initialize
    last_odom_yaw_ = yaw;
    odom_initialized_ = true;
    RCLCPP_INFO(this->get_logger(), "Odometry initialized, initial yaw: %.3f rad", yaw);
  } else {
    // Calculate yaw change (handle wrap-around)
    double delta_yaw = yaw - last_odom_yaw_;

    // Normalize delta_yaw to [-pi, pi]
    while (delta_yaw > M_PI) delta_yaw -= 2.0 * M_PI;
    while (delta_yaw < -M_PI) delta_yaw += 2.0 * M_PI;

    current_yaw_ += delta_yaw;

    // Normalize current_yaw to [-pi, pi]
    while (current_yaw_ > M_PI) current_yaw_ -= 2.0 * M_PI;
    while (current_yaw_ < -M_PI) current_yaw_ += 2.0 * M_PI;

    last_odom_yaw_ = yaw;

    RCLCPP_DEBUG(this->get_logger(),
      "Yaw updated: delta=%.4f, current=%.3f rad", delta_yaw, current_yaw_);
  }
}

void GnssOdomFusion::update_yaw_from_gnss_movement(const nav_msgs::msg::Odometry & current_odom)
{
  // Initialize correction reference on first valid data
  if (!odom_correction_reference_valid_) {
    gnss_x_at_last_correction_ = gnss_x_;
    gnss_y_at_last_correction_ = gnss_y_;
    odom_x_at_last_correction_ = current_odom.pose.pose.position.x;
    odom_y_at_last_correction_ = current_odom.pose.pose.position.y;
    odom_correction_reference_valid_ = true;
    RCLCPP_INFO(this->get_logger(), "Yaw correction reference initialized");
    return;
  }

  // Calculate GNSS movement since last correction
  double gnss_dx = gnss_x_ - gnss_x_at_last_correction_;
  double gnss_dy = gnss_y_ - gnss_y_at_last_correction_;
  double gnss_distance = std::sqrt(gnss_dx * gnss_dx + gnss_dy * gnss_dy);

  // Calculate odometry movement since last correction
  double odom_dx = current_odom.pose.pose.position.x - odom_x_at_last_correction_;
  double odom_dy = current_odom.pose.pose.position.y - odom_y_at_last_correction_;
  double odom_distance = std::sqrt(odom_dx * odom_dx + odom_dy * odom_dy);

  // Check if both movements are significant
  if (gnss_distance < min_gnss_movement_for_yaw_) {
    RCLCPP_DEBUG(this->get_logger(),
      "GNSS movement too small: %.3f m (threshold: %.2f m)",
      gnss_distance, min_gnss_movement_for_yaw_);
    return;
  }

  if (odom_distance < min_odom_movement_for_yaw_) {
    RCLCPP_DEBUG(this->get_logger(),
      "Odom movement too small: %.3f m (threshold: %.2f m)",
      odom_distance, min_odom_movement_for_yaw_);
    return;
  }

  // Check distance consistency between GNSS and odometry
  double distance_diff = std::abs(gnss_distance - odom_distance);
  double distance_avg = (gnss_distance + odom_distance) / 2.0;
  double distance_mismatch_ratio = distance_diff / distance_avg;

  if (distance_mismatch_ratio > max_distance_mismatch_ratio_) {
    RCLCPP_WARN(this->get_logger(),
      "Distance mismatch detected! GNSS: %.3f m, Odom: %.3f m, ratio: %.3f (threshold: %.2f) - Rejecting yaw correction",
      gnss_distance, odom_distance, distance_mismatch_ratio, max_distance_mismatch_ratio_);

    // Reset reference to current position for next attempt
    gnss_x_at_last_correction_ = gnss_x_;
    gnss_y_at_last_correction_ = gnss_y_;
    odom_x_at_last_correction_ = current_odom.pose.pose.position.x;
    odom_y_at_last_correction_ = current_odom.pose.pose.position.y;
    return;
  }

  // Calculate yaw from GNSS movement vector
  double gnss_yaw = std::atan2(gnss_dy, gnss_dx);

  // Calculate angular difference between GNSS yaw and current yaw
  double yaw_diff = gnss_yaw - current_yaw_;

  // Normalize to [-pi, pi]
  while (yaw_diff > M_PI) yaw_diff -= 2.0 * M_PI;
  while (yaw_diff < -M_PI) yaw_diff += 2.0 * M_PI;

  // Apply weighted correction
  double yaw_correction = yaw_diff * gnss_yaw_weight_;
  current_yaw_ += yaw_correction;

  // Normalize current_yaw to [-pi, pi]
  while (current_yaw_ > M_PI) current_yaw_ -= 2.0 * M_PI;
  while (current_yaw_ < -M_PI) current_yaw_ += 2.0 * M_PI;

  RCLCPP_INFO(this->get_logger(),
    "GNSS yaw correction applied: GNSS_dist=%.3f m, Odom_dist=%.3f m, mismatch=%.1f%%, "
    "gnss_yaw=%.3f rad, yaw_diff=%.3f rad, correction=%.3f rad, new_yaw=%.3f rad",
    gnss_distance, odom_distance, distance_mismatch_ratio * 100.0,
    gnss_yaw, yaw_diff, yaw_correction, current_yaw_);

  // Update correction reference to current position
  gnss_x_at_last_correction_ = gnss_x_;
  gnss_y_at_last_correction_ = gnss_y_;
  odom_x_at_last_correction_ = current_odom.pose.pose.position.x;
  odom_y_at_last_correction_ = current_odom.pose.pose.position.y;
}

void GnssOdomFusion::initialpose_callback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg)
{
  // Extract yaw from initialpose quaternion
  tf2::Quaternion q(
    msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z,
    msg->pose.pose.orientation.w);

  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

  // Update current yaw
  current_yaw_ = yaw;

  // Reset odometry reference
  if (odom_initialized_) {
    // Keep odometry initialized but don't reset last_odom_yaw
    // This allows smooth transition
  }

  RCLCPP_INFO(this->get_logger(),
    "Yaw initialized from /initialpose: %.3f rad (%.1f degrees)",
    current_yaw_, current_yaw_ * 180.0 / M_PI);
}

bool GnssOdomFusion::is_gnss_valid() const
{
  if (!gnss_valid_) {
    return false;
  }

  // Check timeout
  auto duration = (this->now() - last_gnss_time_).seconds();
  if (duration > gnss_timeout_) {
    return false;
  }

  return true;
}

void GnssOdomFusion::timer_callback()
{
  if (gnss_received_ && odom_initialized_) {
    publish_tf();
  } else {
    if (!gnss_received_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "Waiting for valid GNSS data...");
    }
    if (!odom_initialized_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "Waiting for odometry data...");
    }
  }
}

void GnssOdomFusion::publish_tf()
{
  auto now = this->now();

  // Create transform
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = now;
  transform.header.frame_id = "map";
  transform.child_frame_id = "base_link";

  // Use GNSS position if available, otherwise stay at last known position
  bool gnss_valid_now = is_gnss_valid();
  if (gnss_valid_now) {
    transform.transform.translation.x = gnss_x_;
    transform.transform.translation.y = gnss_y_;
    transform.transform.translation.z = gnss_z_;
  } else {
    // GNSS lost - continue using last known position with odometry-based orientation
    transform.transform.translation.x = gnss_x_;
    transform.transform.translation.y = gnss_y_;
    transform.transform.translation.z = gnss_z_;

    auto duration = (this->now() - last_gnss_time_).seconds();
    if (duration > gnss_timeout_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "GNSS timeout (%.2f sec) - using last known position with odometry yaw", duration);
    } else {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "GNSS invalid - using last known position with odometry yaw");
    }
  }

  // Use current yaw from odometry integration
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, current_yaw_);
  transform.transform.rotation.x = q.x();
  transform.transform.rotation.y = q.y();
  transform.transform.rotation.z = q.z();
  transform.transform.rotation.w = q.w();

  // Publish TF
  tf_broadcaster_->sendTransform(transform);

  // Publish pose for debugging
  geometry_msgs::msg::PoseStamped pose_msg;
  pose_msg.header = transform.header;
  pose_msg.pose.position.x = transform.transform.translation.x;
  pose_msg.pose.position.y = transform.transform.translation.y;
  pose_msg.pose.position.z = transform.transform.translation.z;
  pose_msg.pose.orientation = transform.transform.rotation;
  pose_pub_->publish(pose_msg);

  RCLCPP_DEBUG(this->get_logger(),
    "TF published: x=%.3f, y=%.3f, yaw=%.3f, gnss_valid=%d",
    transform.transform.translation.x,
    transform.transform.translation.y,
    current_yaw_,
    is_gnss_valid());
}

}  // namespace cub_bringup

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<cub_bringup::GnssOdomFusion>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
