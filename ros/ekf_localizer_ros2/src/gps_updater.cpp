#include "gps_updater/gps_updater.h"
#include <iostream>
#include <iomanip>
#include <chrono>

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
    this->declare_parameter<std::string>("initialpose_topic_name", "/initialpose");

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
    this->get_parameter("initialpose_topic_name", initialpose_topic_name_);

    // Subscriber
    gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        gps_topic_name_, rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::BestEffort),
        std::bind(&GpsUpdater::gps_callback, this, std::placeholders::_1));
    initialpose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        initialpose_topic_name_, rclcpp::QoS(1).reliable(),
        std::bind(&GpsUpdater::initialpose_callback, this, std::placeholders::_1));

    // Publisher
    gps_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        gps_pose_topic_name_, rclcpp::QoS(10).reliable());

    is_gps_initialized_ = false;

    // Manual origin setting
    if(use_manual_origin_ && gps_origin_lat_ != 0.0 && gps_origin_lon_ != 0.0) {
        gps_projector_ = std::make_unique<GeographicLib::LocalCartesian>(
            gps_origin_lat_, gps_origin_lon_, gps_origin_alt_);
        is_gps_initialized_ = true;
        
        std::cout << "========================================" << std::endl;
        std::cout << "GPS ORIGIN - MANUAL MODE" << std::endl;
        std::cout << "  Latitude  : " << std::fixed << std::setprecision(8) 
                  << gps_origin_lat_ << "°" << std::endl;
        std::cout << "  Longitude : " << std::fixed << std::setprecision(8) 
                  << gps_origin_lon_ << "°" << std::endl;
        std::cout << "  Altitude  : " << std::fixed << std::setprecision(2) 
                  << gps_origin_alt_ << " m" << std::endl;
        std::cout << "This position is (0, 0, 0) in map frame" << std::endl;
        std::cout << "========================================" << std::endl;
    } else {
        std::cout << "========================================" << std::endl;
        std::cout << "GPS ORIGIN - AUTOMATIC MODE" << std::endl;
        std::cout << "First valid GPS position will be set as origin (0, 0, 0)" << std::endl;
        std::cout << "========================================" << std::endl;
    }

    std::cout << "GPS Updater initialized" << std::endl;
    std::cout << "  Min satellites    : " << std::fixed << std::setprecision(0) 
              << min_satellites_ << std::endl;
    std::cout << "  Max HDOP          : " << std::fixed << std::setprecision(1) 
              << max_hdop_ << std::endl;
    std::cout << "  Max covariance    : " << std::fixed << std::setprecision(1) 
              << max_covariance_threshold_ << " m" << std::endl;
}

GpsUpdater::~GpsUpdater() {}

void GpsUpdater::initialpose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg)
{
    std::cout << "Received /initialpose" << std::endl;
    
    if(use_manual_origin_) {
        return;
    }
    
    // 自動原点モードの場合のみGPS原点をリセット
    std::cout << "  Automatic origin mode: Resetting GPS origin" << std::endl;
    reset_gps_origin();
}

void GpsUpdater::reset_gps_origin()
{
    // use_manual_origin_のチェックを削除（呼び出し側で制御）
    
    // GPS初期化フラグをリセット
    is_gps_initialized_ = false;
    
    // GPS projectorをリセット
    gps_projector_.reset();
    
    // 原点情報をクリア
    gps_origin_lat_ = 0.0;
    gps_origin_lon_ = 0.0;
    gps_origin_alt_ = 0.0;
    
    std::cout << "========================================" << std::endl;
    std::cout << "GPS ORIGIN RESET" << std::endl;
    std::cout << "Next valid GPS message will set new origin (0, 0, 0)" << std::endl;
    std::cout << "========================================" << std::endl;
}

void GpsUpdater::gps_callback(const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg)
{
    current_gps_ = *msg;

    // Check GPS quality
    if(!check_gps_quality(current_gps_)) {
        static auto last_warn_time = std::chrono::steady_clock::now();
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - last_warn_time).count();
        
        if(elapsed >= 5) {
            std::cout << "GPS quality check failed" << std::endl;
            last_warn_time = now;
        }
        return;
    }

    // Initialize GPS origin from first valid message (自動モードのみ)
    if(!is_gps_initialized_) {
        gps_origin_lat_ = msg->latitude;
        gps_origin_lon_ = msg->longitude;
        gps_origin_alt_ = msg->altitude;
        
        gps_projector_ = std::make_unique<GeographicLib::LocalCartesian>(
            gps_origin_lat_, gps_origin_lon_, gps_origin_alt_);
        
        is_gps_initialized_ = true;
        
        std::cout << "========================================" << std::endl;
        std::cout << "GPS ORIGIN INITIALIZED (AUTOMATIC)" << std::endl;
        std::cout << "  Latitude  : " << std::fixed << std::setprecision(8) 
                  << gps_origin_lat_ << "°" << std::endl;
        std::cout << "  Longitude : " << std::fixed << std::setprecision(8) 
                  << gps_origin_lon_ << "°" << std::endl;
        std::cout << "  Altitude  : " << std::fixed << std::setprecision(2) 
                  << gps_origin_alt_ << " m" << std::endl;
        std::cout << "This GPS position is now (0, 0, 0) in map frame" << std::endl;
        std::cout << "========================================" << std::endl;
        return;
    }

    // Convert and publish GPS pose
    publish_gps_pose();
}

/**
 * @brief GPS品質をチェック（詳細なログ出力付き）
 * @param gps_msg GPSメッセージ
 * @return true: 品質OK, false: 品質NG
 */
bool GpsUpdater::check_gps_quality(const sensor_msgs::msg::NavSatFix& gps_msg)
{
    // ===== ステータスチェック =====
    int status = gps_msg.status.status;
    std::string status_name = get_gps_status_name(status);
    
    // SBAS_FIX以上だけ信頼
    if(status < sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX) {  // 1以上
        static auto last_warn_time = std::chrono::steady_clock::now();
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - last_warn_time).count();
        
        if(elapsed >= 5) {
            std::cout << "GPS: No valid fix (status: " << status << " [" 
                      << status_name << "])" << std::endl;
            last_warn_time = now;
        }
        return false;
    }

    // ===== NAN確認 =====
    if(std::isnan(gps_msg.latitude) || std::isnan(gps_msg.longitude) || std::isnan(gps_msg.altitude)) {
        static auto last_warn_time = std::chrono::steady_clock::now();
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - last_warn_time).count();
        
        if(elapsed >= 5) {
            std::cout << "GPS: Invalid coordinates (NaN detected)" << std::endl;
            last_warn_time = now;
        }
        return false;
    }

    // ===== 座標が範囲内か確認 =====
    if(std::abs(gps_msg.latitude) > 90.0 || std::abs(gps_msg.longitude) > 180.0) {
        static auto last_warn_time = std::chrono::steady_clock::now();
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - last_warn_time).count();
        
        if(elapsed >= 5) {
            std::cout << "GPS: Coordinates out of valid range (lat: " 
                      << std::fixed << std::setprecision(2) << gps_msg.latitude 
                      << ", lon: " << gps_msg.longitude << ")" << std::endl;
            last_warn_time = now;
        }
        return false;
    }

    // ===== 共分散のタイプチェック =====
    // KNOWN(3), DIAGONAL_KNOWN(2), APPROXIMATED(1) を受け入れる
    int cov_type = gps_msg.position_covariance_type;
    if(cov_type != sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_KNOWN &&
       cov_type != sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_APPROXIMATED &&
       cov_type != sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN) {
        static auto last_warn_time = std::chrono::steady_clock::now();
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - last_warn_time).count();
        
        if(elapsed >= 5) {
            std::cout << "GPS: Covariance type not reliable (type: " << cov_type 
                      << "). Only KNOWN(3), DIAGONAL_KNOWN(2), or APPROXIMATED(1) accepted" 
                      << std::endl;
            last_warn_time = now;
        }
        return false;
    }

    // ===== 水平精度の確認 =====
    double var_x = gps_msg.position_covariance[0];
    double var_y = gps_msg.position_covariance[4];
    double std_dev_x = std::sqrt(var_x);
    double std_dev_y = std::sqrt(var_y);
    double horizontal_std_dev = std::sqrt(std_dev_x * std_dev_x + std_dev_y * std_dev_y);
    
    if(horizontal_std_dev > max_covariance_threshold_) {
        static auto last_warn_time = std::chrono::steady_clock::now();
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - last_warn_time).count();
        
        if(elapsed >= 5) {
            std::cout << "GPS: Horizontal accuracy too low (σ_h: " 
                      << std::fixed << std::setprecision(2) << horizontal_std_dev 
                      << " m, threshold: " << max_covariance_threshold_ << " m)" 
                      << std::endl;
            last_warn_time = now;
        }
        return false;
    }
    
    // ===== 品質OK - 詳細情報をログ出力（10秒に1回） =====
    static auto last_info_time = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - last_info_time).count();
    
    if(elapsed >= 10) {
        std::cout << "GPS Quality: ✓ GOOD" << std::endl;
        std::cout << "  Status       : " << status << " [" << status_name << "]" << std::endl;
        std::cout << "  Cov Type     : " << cov_type << " [" 
                  << get_covariance_type_name(cov_type) << "]" << std::endl;
        std::cout << "  σ_x          : " << std::fixed << std::setprecision(3) 
                  << std_dev_x << " m" << std::endl;
        std::cout << "  σ_y          : " << std::fixed << std::setprecision(3) 
                  << std_dev_y << " m" << std::endl;
        std::cout << "  σ_horizontal : " << std::fixed << std::setprecision(3) 
                  << horizontal_std_dev << " m" << std::endl;
        last_info_time = now;
    }

    return true;
}

/**
 * @brief GPSステータスの名前を取得
 * @param status ステータス値
 * @return ステータス名
 */
std::string GpsUpdater::get_gps_status_name(int status)
{
    switch(status) {
        case sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX:
            return "NO_FIX";
        case sensor_msgs::msg::NavSatStatus::STATUS_FIX:
            return "FIX";
        case sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX:
            return "SBAS_FIX";
        case sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX:
            return "GBAS_FIX";
        default:
            // RTK-FIX などカスタムステータスの場合
            if(status >= 4) {
                return "HIGH_PRECISION_FIX";
            }
            return "UNKNOWN";
    }
}

/**
 * @brief 共分散タイプの名前を取得
 * @param cov_type 共分散タイプ値
 * @return タイプ名
 */
std::string GpsUpdater::get_covariance_type_name(int cov_type)
{
    switch(cov_type) {
        case sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN:
            return "UNKNOWN";
        case sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_APPROXIMATED:
            return "APPROXIMATED";
        case sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN:
            return "DIAGONAL_KNOWN";
        case sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_KNOWN:
            return "KNOWN";
        default:
            return "INVALID";
    }
}

void GpsUpdater::convert_gps_to_local(double lat, double lon, double alt,
                                       double& x, double& y, double& z)
{
    if(!gps_projector_) {
        std::cout << "❌ ERROR: GPS projector not initialized" << std::endl;
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

    // Covariance matrix (6x6) - 初期化
    std::fill(gps_pose_.pose.covariance.begin(), 
              gps_pose_.pose.covariance.end(), 0.0);

    // ===== 共分散情報の転送（重要！） =====
    if(current_gps_.position_covariance_type == sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_KNOWN ||
       current_gps_.position_covariance_type == sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_APPROXIMATED ||
       current_gps_.position_covariance_type == sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN) {
        
        // GPS共分散をそのまま使用（EKF側で品質判定に使用される）
        gps_pose_.pose.covariance[0] = current_gps_.position_covariance[0];   // x variance
        gps_pose_.pose.covariance[1] = current_gps_.position_covariance[1];   // xy covariance
        gps_pose_.pose.covariance[6] = current_gps_.position_covariance[3];   // xy covariance
        gps_pose_.pose.covariance[7] = current_gps_.position_covariance[4];   // y variance
        gps_pose_.pose.covariance[14] = current_gps_.position_covariance[8];  // z variance
        
        // デバッグ: 共分散値をログ出力
        double sigma_x = std::sqrt(gps_pose_.pose.covariance[0]);
        double sigma_y = std::sqrt(gps_pose_.pose.covariance[7]);
        double sigma_h = std::sqrt(sigma_x * sigma_x + sigma_y * sigma_y);
        
        std::cout << "[DEBUG] GPS covariance transferred: σ_x=" << std::fixed 
                  << std::setprecision(3) << sigma_x << " m, σ_y=" << sigma_y 
                  << " m, σ_h=" << sigma_h << " m" << std::endl;
        
    } else {
        // 共分散が無効な場合はデフォルト値（3m標準偏差）
        gps_pose_.pose.covariance[0] = 9.0;   // x variance (3m std)
        gps_pose_.pose.covariance[7] = 9.0;   // y variance (3m std)
        gps_pose_.pose.covariance[14] = 9.0;  // z variance (3m std)
        
        static auto last_warn_time = std::chrono::steady_clock::now();
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - last_warn_time).count();
        
        if(elapsed >= 5) {
            std::cout << "GPS covariance not available - using default (σ=3.0m)" << std::endl;
            last_warn_time = now;
        }
    }

    // Large variance for orientation (GPS doesn't provide orientation)
    gps_pose_.pose.covariance[21] = 1e6;  // roll variance
    gps_pose_.pose.covariance[28] = 1e6;  // pitch variance
    gps_pose_.pose.covariance[35] = 1e6;  // yaw variance

    // Publish
    gps_pose_pub_->publish(gps_pose_);

    std::cout << "[DEBUG] GPS pose published: x=" << std::fixed << std::setprecision(2) 
              << x << ", y=" << y << ", z=" << z << std::endl;
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
