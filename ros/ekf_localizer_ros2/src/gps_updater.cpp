#include "gps_updater/gps_updater.h"

GpsUpdater::GpsUpdater() : Node("GpsUpdater")
{
    // Parameters declaration
    this->declare_parameter<std::string>("gps_topic_name", "/fix");
    this->declare_parameter<std::string>("gps_pose_topic_name", "/gps_pose");
    this->declare_parameter<std::string>("map_frame_id", "map");
    this->declare_parameter<double>("gps_origin_lat", 0.0);
    this->declare_parameter<double>("gps_origin_lon", 0.0);
    this->declare_parameter<double>("gps_origin_alt", 0.0);
    this->declare_parameter<bool>("use_manual_origin", false);
    this->declare_parameter<double>("min_satellites", 6.0);
    this->declare_parameter<double>("max_hdop", 3.0);
    this->declare_parameter<double>("max_covariance_threshold", 10.0);

    // Get parameters
    this->get_parameter("gps_topic_name", gps_topic_name_);
    this->get_parameter("gps_pose_topic_name", gps_pose_topic_name_);
    this->get_parameter("map_frame_id", map_frame_id_);
    this->get_parameter("gps_origin_lat", gps_origin_lat_);
    this->get_parameter("gps_origin_lon", gps_origin_lon_);
    this->get_parameter("gps_origin_alt", gps_origin_alt_);
    this->get_parameter("use_manual_origin", use_manual_origin_);
    this->get_parameter("min_satellites", min_satellites_);
    this->get_parameter("max_hdop", max_hdop_);
    this->get_parameter("max_covariance_threshold", max_covariance_threshold_);

    // Subscriber
    gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        gps_topic_name_, rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::BestEffort),
        std::bind(&GpsUpdater::gps_callback, this, std::placeholders::_1));

    // Publisher
    gps_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        gps_pose_topic_name_, rclcpp::QoS(10).reliable());

    is_gps_initialized_ = false;

    // Manual origin setting
    if(use_manual_origin_ && gps_origin_lat_ != 0.0 && gps_origin_lon_ != 0.0) {
        gps_projector_ = std::make_unique<GeographicLib::LocalCartesian>(
            gps_origin_lat_, gps_origin_lon_, gps_origin_alt_);
        is_gps_initialized_ = true;
        
        RCLCPP_INFO(this->get_logger(), 
            "GPS origin manually set to: lat=%.8f, lon=%.8f, alt=%.2f",
            gps_origin_lat_, gps_origin_lon_, gps_origin_alt_);
    }

    RCLCPP_INFO(this->get_logger(), "GPS Updater initialized");
    RCLCPP_INFO(this->get_logger(), "  Min satellites    : %.0f", min_satellites_);
    RCLCPP_INFO(this->get_logger(), "  Max HDOP          : %.1f", max_hdop_);
    RCLCPP_INFO(this->get_logger(), "  Max covariance    : %.1f m", max_covariance_threshold_);
}

GpsUpdater::~GpsUpdater() {}

void GpsUpdater::gps_callback(const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg)
{
    current_gps_ = *msg;

    // Check GPS quality
    if(!check_gps_quality(current_gps_)) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "GPS quality check failed");
        return;
    }

    // Initialize GPS origin from first valid message
    if(!is_gps_initialized_) {
        gps_origin_lat_ = msg->latitude;
        gps_origin_lon_ = msg->longitude;
        gps_origin_alt_ = msg->altitude;
        
        gps_projector_ = std::make_unique<GeographicLib::LocalCartesian>(
            gps_origin_lat_, gps_origin_lon_, gps_origin_alt_);
        
        is_gps_initialized_ = true;
        
        RCLCPP_INFO(this->get_logger(), 
            "GPS origin initialized: lat=%.8f, lon=%.8f, alt=%.2f",
            gps_origin_lat_, gps_origin_lon_, gps_origin_alt_);
        return;
    }

    // Convert and publish GPS pose
    publish_gps_pose();
}

bool GpsUpdater::check_gps_quality(const sensor_msgs::msg::NavSatFix& gps_msg)
{
    // SBAS_FIX以上だけ信頼
    if(gps_msg.status.status < sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX){  // 1以上
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "GPS: No fix (status: %d)", gps_msg.status.status);
        return false;
    }

    // NAN確認
    if(std::isnan(gps_msg.latitude) || std::isnan(gps_msg.longitude) || std::isnan(gps_msg.altitude)) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "GPS: Invalid coordinates (NaN detected)");
        return false;
    }

    // 座標が範囲内か確認
    if(std::abs(gps_msg.latitude) > 90.0 || std::abs(gps_msg.longitude) > 180.0) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "GPS: Coordinates out of valid range (lat: %.2f, lon: %.2f)",
            gps_msg.latitude, gps_msg.longitude);
        return false;
    }

    // 共分散のタイプがKNOWNかAPPROXIMATEDの時だけ採用
    if(gps_msg.position_covariance_type != sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_KNOWN &&
       gps_msg.position_covariance_type != sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_APPROXIMATED) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "GPS: Covariance type not reliable (type: %d). Only KNOWN(3) or APPROXIMATED(1) accepted",
            gps_msg.position_covariance_type);
        return false;
    }

    // 水平精度の確認
    double std_dev_x = std::sqrt(gps_msg.position_covariance[0]);
    double std_dev_y = std::sqrt(gps_msg.position_covariance[4]);
    double horizontal_std_dev = std::sqrt(std_dev_x * std_dev_x + std_dev_y * std_dev_y);
    
    if(horizontal_std_dev > max_covariance_threshold_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "GPS: Horizontal accuracy too low (std_dev: %.2f m, threshold: %.2f m)",
            horizontal_std_dev, max_covariance_threshold_);
        return false;
    }
    
    RCLCPP_DEBUG(this->get_logger(), 
        "GPS: Good quality (covariance_type: %d, std_dev: %.2f m)", 
        gps_msg.position_covariance_type, horizontal_std_dev);

    return true;
}

void GpsUpdater::convert_gps_to_local(double lat, double lon, double alt,
                                       double& x, double& y, double& z)
{
    if(!gps_projector_) {
        RCLCPP_ERROR(this->get_logger(), "GPS projector not initialized");
        return;
    }
    
    gps_projector_->Forward(lat, lon, alt, x, y, z);
}

void GpsUpdater::publish_gps_pose()
{
    // Convert GPS to local coordinates
    double x, y, z;
    convert_gps_to_local(current_gps_.latitude, 
                        current_gps_.longitude, 
                        current_gps_.altitude,
                        x, y, z);

    // Create PoseWithCovarianceStamped message
    gps_pose_.header.stamp = current_gps_.header.stamp;
    gps_pose_.header.frame_id = map_frame_id_;

    // Position
    gps_pose_.pose.pose.position.x = x;
    gps_pose_.pose.pose.position.y = y;
    gps_pose_.pose.pose.position.z = z;

    // Orientation (GPS doesn't provide orientation, set identity quaternion)
    gps_pose_.pose.pose.orientation.x = 0.0;
    gps_pose_.pose.pose.orientation.y = 0.0;
    gps_pose_.pose.pose.orientation.z = 0.0;
    gps_pose_.pose.pose.orientation.w = 1.0;

    // Covariance matrix (6x6)
    std::fill(gps_pose_.pose.covariance.begin(), 
              gps_pose_.pose.covariance.end(), 0.0);

    if(current_gps_.position_covariance_type == sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_KNOWN ||
       current_gps_.position_covariance_type == sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_APPROXIMATED) {
        // Copy GPS covariance for x, y, z
        gps_pose_.pose.covariance[0] = current_gps_.position_covariance[0];   // x variance
        gps_pose_.pose.covariance[1] = current_gps_.position_covariance[1];   // xy covariance
        gps_pose_.pose.covariance[6] = current_gps_.position_covariance[3];   // xy covariance
        gps_pose_.pose.covariance[7] = current_gps_.position_covariance[4];   // y variance
        gps_pose_.pose.covariance[14] = current_gps_.position_covariance[8];  // z variance
    } else {
        // Default covariance if not provided (3m standard deviation)
        gps_pose_.pose.covariance[0] = 9.0;   // x variance
        gps_pose_.pose.covariance[7] = 9.0;   // y variance
        gps_pose_.pose.covariance[14] = 9.0;  // z variance
    }

    // Large variance for orientation (GPS doesn't provide orientation)
    gps_pose_.pose.covariance[21] = 1e6;  // roll variance
    gps_pose_.pose.covariance[28] = 1e6;  // pitch variance
    gps_pose_.pose.covariance[35] = 1e6;  // yaw variance

    // Publish
    gps_pose_pub_->publish(gps_pose_);

    RCLCPP_DEBUG(this->get_logger(), 
        "GPS pose published: x=%.2f, y=%.2f, z=%.2f", x, y, z);
}

int main(int argc, char** argv)
{
    std::cout << "---gps_updater---" << std::endl;
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GpsUpdater>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
