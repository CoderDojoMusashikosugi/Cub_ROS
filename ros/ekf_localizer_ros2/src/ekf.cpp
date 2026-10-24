#include "ekf/ekf.h"


// TODO NAN対策
EKF::EKF() : Node("EKF")
{
	this->declare_parameter<std::string>("ndt_pose_topic_name", "/test/ndt_pose");
    this->declare_parameter<std::string>("imu_topic_name", "/bno055/imu");
    this->declare_parameter<std::string>("odom_topic_name", "/odom");
    this->declare_parameter<std::string>("ekf_pose_topic_name", "/test/ekf_pose");
    this->declare_parameter<std::string>("map_frame_id", "map");
    this->declare_parameter<std::string>("odom_frame_id", "odom");
    this->declare_parameter<std::string>("base_link_frame_id", "base_link");
    this->declare_parameter<bool>("is_odom_tf", false);
    this->declare_parameter<bool>("GPS_MEASUREMENT_ENABLE", false);
    this->declare_parameter<bool>("NDT_MEASUREMENT_ENABLE", false);
    this->declare_parameter<bool>("gps_respawn_enable", false);
	this->declare_parameter<double>("INIT_X", {0.0});
	this->declare_parameter<double>("INIT_Y", {0.0});
	this->declare_parameter<double>("INIT_Z", {0.0});
	this->declare_parameter<double>("INIT_ROLL", {0.0});
	this->declare_parameter<double>("INIT_PITCH", {0.0});
	this->declare_parameter<double>("INIT_YAW", {0.0});
	this->declare_parameter<double>("INIT_SIGMA", {1e-3});
	this->declare_parameter<double>("SIGMA_IMU", {1e-3});
	this->declare_parameter<double>("SIGMA_ODOM", {1e-3});
	this->declare_parameter<double>("SIGMA_NDT", {1e-3});
	this->declare_parameter<double>("MOTION_NOISE_NN", {1e-3});
	this->declare_parameter<double>("MOTION_NOISE_NO", {1e-3});
	this->declare_parameter<double>("MOTION_NOISE_ON", {1e-3});
	this->declare_parameter<double>("MOTION_NOISE_OO", {1e-3});
	this->declare_parameter<double>("TH_MAHALANOBIS", 1.5);
	this->declare_parameter<double>("TH_COVARIANCE", 1.0);
	this->declare_parameter<double>("TH_POSE_COVARIANCE", 1.0);
	this->declare_parameter<double>("TH_DIRECTION_COVARIANCE", 1.0);
	this->declare_parameter<std::string>("gps_pose_topic_name", "/gps_pose");
	this->declare_parameter<double>("SIGMA_GPS", 3.0);
	this->declare_parameter<double>("EKF_HZ", 30.0);
	this->declare_parameter<std::string>("initialpose_topic_name", "/initialpose");
    this->declare_parameter<double>("measurement_suppress_duration", 2.0);
	// GPS高精度リスポーン関連のパラメータ
	this->declare_parameter<double>("gps_high_confidence_threshold", 1.0);
	this->declare_parameter<int>("gps_high_quality_count_threshold", 3);
	this->declare_parameter<double>("gps_respawn_distance_threshold", 3.0);
	this->declare_parameter<double>("gps_respawn_max_distance", 50.0);
    this->declare_parameter<double>("MAX_ODOM_VELOCITY", 3.0);
    this->declare_parameter<double>("MAX_ODOM_ANGULAR_VELOCITY", 2.0);
    this->declare_parameter<double>("MAX_IMU_ANGULAR_VELOCITY", 2.0);
    this->declare_parameter<double>("MAX_NDT_POSITION_JUMP", 5.0);
    this->declare_parameter<double>("MAX_NDT_YAW_JUMP_DEG", 90.0);

    // Retrieve the parameters
    this->get_parameter("ndt_pose_topic_name", ndt_pose_topic_name_);
    this->get_parameter("imu_topic_name", imu_topic_name_);
    this->get_parameter("odom_topic_name", odom_topic_name_);
    this->get_parameter("ekf_pose_topic_name", ekf_pose_topic_name_);
    this->get_parameter("map_frame_id", map_frame_id_);
    this->get_parameter("odom_frame_id", odom_frame_id_);
    this->get_parameter("base_link_frame_id", base_link_frame_id_);
    this->get_parameter("is_odom_tf", is_odom_tf_);
    this->get_parameter("GPS_MEASUREMENT_ENABLE", gps_measurement_enable_);
    this->get_parameter("NDT_MEASUREMENT_ENABLE", ndt_measurement_enable_);
    this->get_parameter("gps_respawn_enable", gps_respawn_enable_);
	this->get_parameter("INIT_X", INIT_X_);
	this->get_parameter("INIT_Y", INIT_Y_);
	this->get_parameter("INIT_Z", INIT_Z_);
	this->get_parameter("INIT_ROLL", INIT_ROLL_);
	this->get_parameter("INIT_PITCH", INIT_PITCH_);
	this->get_parameter("INIT_YAW", INIT_YAW_);
	this->get_parameter("INIT_SIGMA", INIT_SIGMA_);
	this->get_parameter("SIGMA_IMU", SIGMA_IMU_);
	this->get_parameter("SIGMA_ODOM", SIGMA_ODOM_);
	this->get_parameter("SIGMA_NDT", SIGMA_NDT_);
	this->get_parameter("MOTION_NOISE_NN", MOTION_NOISE_NN_);
	this->get_parameter("MOTION_NOISE_NO", MOTION_NOISE_NO_);
	this->get_parameter("MOTION_NOISE_ON", MOTION_NOISE_ON_);
	this->get_parameter("MOTION_NOISE_OO", MOTION_NOISE_OO_);

	this->get_parameter("TH_MAHALANOBIS", th_mahalanobis_);
	this->get_parameter("TH_COVARIANCE", th_covariance_);
	this->get_parameter("TH_POSE_COVARIANCE", th_pose_covariance_);
	this->get_parameter("TH_DIRECTION_COVARIANCE", th_direction_covariance_);
	this->get_parameter("gps_pose_topic_name", gps_pose_topic_name_);
	this->get_parameter("SIGMA_GPS", SIGMA_GPS_);
	this->get_parameter("EKF_HZ", ekf_hz_);
    this->get_parameter("initialpose_topic_name", initialpose_topic_name_);
    this->get_parameter("measurement_suppress_duration", measurement_suppress_duration_);
	// GPS高精度リスポーン関連
	this->get_parameter("gps_high_confidence_threshold", gps_high_confidence_threshold_);
	this->get_parameter("gps_high_quality_count_threshold", gps_high_quality_count_threshold_);
	this->get_parameter("gps_respawn_distance_threshold", gps_respawn_distance_threshold_);
	this->get_parameter("gps_respawn_max_distance", gps_respawn_max_distance_);

    this->get_parameter("MAX_ODOM_VELOCITY", max_odom_velocity_);
    this->get_parameter("MAX_ODOM_ANGULAR_VELOCITY", max_odom_angular_velocity_);
    this->get_parameter("MAX_IMU_ANGULAR_VELOCITY", max_imu_angular_velocity_);
    this->get_parameter("MAX_NDT_POSITION_JUMP", max_ndt_position_jump_);
    this->get_parameter("MAX_NDT_YAW_JUMP_DEG", max_ndt_yaw_jump_deg_);

    ndt_pose_sub_  = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        ndt_pose_topic_name_, rclcpp::QoS(1).reliable(),
        std::bind(&EKF::ndt_pose_callback, this, std::placeholders::_1));
    imu_sub_  = this->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_name_, rclcpp::QoS(1).reliable(),
        std::bind(&EKF::imu_callback, this, std::placeholders::_1));
    odom_sub_  = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_name_, rclcpp::QoS(1).reliable(),
        std::bind(&EKF::odom_callback, this, std::placeholders::_1));
	gps_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
		gps_pose_topic_name_, rclcpp::QoS(1).reliable(),
		std::bind(&EKF::gps_pose_callback, this, std::placeholders::_1));
    initialpose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        initialpose_topic_name_,rclcpp::QoS(1).reliable(),
        std::bind(&EKF::initialpose_callback, this, std::placeholders::_1));

    ekf_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        ekf_pose_topic_name_, rclcpp::QoS(1).reliable());
	// path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
	// 	"/ekf_pose/trajectory", rclcpp::QoS(1).reliable());

	auto clock = this->get_clock();

	initialize(INIT_X_, INIT_Y_, INIT_Z_, INIT_ROLL_, INIT_PITCH_, INIT_YAW_);
	is_measurement_.data = false;
	has_received_gps_ = false;
	// GPS高精度リスポーン関連の初期化
	gps_high_quality_count_ = 0;
	gps_respawn_in_progress_ = false;
	
	// last_time_ = this->get_clock()->now();
	ekf_pose_trajectry.header.frame_id = "map"; 
	std::cout << "\n[THRESHOLD PARAMETERS]" << std::endl;
    std::cout << "  TH_MAHALANOBIS          : " << std::fixed << th_mahalanobis_ << std::endl;
    std::cout << "  TH_COVARIANCE           : " << std::fixed  << th_covariance_ << std::endl;
    std::cout << "  TH_POSE_COVARIANCE      : " << std::fixed << th_pose_covariance_ << std::endl;
    std::cout << "  TH_DIRECTION_COVARIANCE : " << std::fixed << th_direction_covariance_ << std::endl;
	std::cout << "  SIGMA_GPS               : " << std::fixed << SIGMA_GPS_ << std::endl;
	std::cout << "  GPS_MEASUREMENT_ENABLE  : " << gps_measurement_enable_ << std::endl;
	std::cout << "  NDT_MEASUREMENT_ENABLE  : " << ndt_measurement_enable_ << std::endl;
	std::cout << "\n[GPS RESPAWN PARAMETERS]" << std::endl;
	std::cout << "  GPS_HIGH_CONF_TH        : " << gps_high_confidence_threshold_ << " m" << std::endl;
	std::cout << "  GPS_QUALITY_COUNT_TH    : " << gps_high_quality_count_threshold_ << " times" << std::endl;
	std::cout << "  GPS_RESPAWN_DIST_TH     : " << gps_respawn_distance_threshold_ << " m" << std::endl;
	std::cout << "  GPS_RESPAWN_MAX_DIST    : " << gps_respawn_max_distance_ << " m" << std::endl;
    std::cout <<"  GPS_RESPAWN_ENABLE      : " << gps_respawn_enable_ << std::endl;

    std::cout << "\n[MOTION UPDATE VALIDATION]" << std::endl;
    std::cout << "  MAX_ODOM_VELOCITY       : " << max_odom_velocity_ << " m/s" << std::endl;
    std::cout << "  MAX_ODOM_ANGULAR_VEL    : " << max_odom_angular_velocity_ << " rad/s" << std::endl;
    std::cout << "  MAX_IMU_ANGULAR_VEL     : " << max_imu_angular_velocity_ << " rad/s" << std::endl;
    
    std::cout << "\n[OBSERVATION UPDATE VALIDATION]" << std::endl;
    std::cout << "  MAX_NDT_POSITION_JUMP   : " << max_ndt_position_jump_ << " m" << std::endl;
    std::cout << "  MAX_NDT_YAW_JUMP        : " << max_ndt_yaw_jump_deg_ << " deg" << std::endl;

}

EKF::~EKF() {}

void EKF::initialize(double x, double y, double z, double roll, double pitch, double yaw)
{
	STATE_SIZE_ = 3;
	X_.setZero(STATE_SIZE_);
	P_.setZero(STATE_SIZE_, STATE_SIZE_);
	P_ = INIT_SIGMA_*Eigen::MatrixXd::Identity(STATE_SIZE_, STATE_SIZE_);
	set_pose(x, y, z, roll, pitch, yaw);
}

void EKF::set_pose(double x, double y, double z, double roll, double pitch, double yaw)
{
	if(X_.size() != STATE_SIZE_){
		std::cout << "No matching STATE SIZE" << std::endl;
		return;
	}
	X_(0) = x;
	X_(1) = y;
	X_(2) = yaw;
}

/**
 * @brief EKFを完全にリセットして指定位置から再スタート
 * @param x 初期x座標 [m]
 * @param y 初期y座標 [m]
 * @param yaw 初期ヨー角 [rad]
 */
void EKF::reset_ekf(double x, double y, double yaw)
{
    RCLCPP_INFO(this->get_logger(), "=== COMPLETE EKF RESET ===");
    
    // 1. 状態ベクトルと共分散を初期化
    X_.setZero(STATE_SIZE_);
    P_.setZero(STATE_SIZE_, STATE_SIZE_);
    P_ = INIT_SIGMA_ * Eigen::MatrixXd::Identity(STATE_SIZE_, STATE_SIZE_);
    
    // 2. 指定位置に設定
    X_(0) = x;
    X_(1) = y;
    X_(2) = yaw;
    
    RCLCPP_INFO(this->get_logger(), "State initialized: x=%.3f, y=%.3f, yaw=%.3f [rad]", x, y, yaw);
    
    // 3. 全てのセンサー受信フラグをリセット
    has_received_odom_ = false;
    has_received_imu_ = false;
    has_received_ndt_pose_ = false;
    has_received_gps_ = false;
    
    // 4. 初回受信フラグをリセット
    is_first_odom_ = true;
    is_first_imu_ = true;
    first_callback_ = true;
    
    RCLCPP_INFO(this->get_logger(), "All sensor flags reset to initial state");
    
    // 5. オドメトリ関連の履歴をクリア
    last_odom_eigen = Eigen::Vector3d::Zero();
    last_odom_pose_ = Eigen::Vector3d::Zero();
    last_odom_yaw_ = 0.0;
    
    // 6. オドメトリメッセージをクリア
    odom_ = nav_msgs::msg::Odometry();
    last_odom_pose = nav_msgs::msg::Odometry();
    
    // 7. IMUメッセージをクリア
    imu_ = sensor_msgs::msg::Imu();
    
    // 8. NDT・GPSメッセージをクリア
    ndt_pose_ = geometry_msgs::msg::PoseStamped();
    gps_pose_ = geometry_msgs::msg::PoseWithCovarianceStamped();
    
    // 9. 測定フラグをリセット
    is_measurement_.data = false;
    
    // 10. タイムスタンプを現在時刻にリセット
    rclcpp::Time now = this->now();
    time_publish_ = now;
    last_time_odom_ = now;
    last_time_imu_ = now;
    now_time_odom_ = now;
    now_time_imu_ = now;
    
    // 11. 軌跡データをクリア（もし使用している場合）
    poses_.clear();
    
    // 12. 測定更新を一定時間抑制
    suppress_measurements_after_reset_ = true;
    measurement_suppress_until_ = now + rclcpp::Duration::from_seconds(measurement_suppress_duration_);
    
    // 13. GPS高精度カウンタをリセット
    gps_high_quality_count_ = 0;
    
    RCLCPP_INFO(this->get_logger(), "All history and timestamps cleared");
    RCLCPP_WARN(this->get_logger(), "  NDT and GPS measurements SUPPRESSED for %.1f seconds", measurement_suppress_duration_);
    RCLCPP_INFO(this->get_logger(), "Using only odometry and IMU during suppression period");
    RCLCPP_INFO(this->get_logger(), "=== EKF RESET COMPLETE ===");
}

/**
 * @brief RVizの2D Pose Estimateからの初期位置設定コールバック
 * @param msg 初期位置メッセージ
 */
void EKF::initialpose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "========================================");
    RCLCPP_INFO(this->get_logger(), "📍 Received initial pose from RViz 2D Pose Estimate");
    
    // クォータニオンからyawのみを計算
    double yaw = calc_yaw_from_quat(msg->pose.pose.orientation);
    
    // 受信した位置情報を取得 (x, yのみ使用、zは無視)
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    
    RCLCPP_INFO(this->get_logger(), 
                "Resetting EKF to:");
    RCLCPP_INFO(this->get_logger(),
                "  Position: x=%.3f, y=%.3f [m]", x, y);
    RCLCPP_INFO(this->get_logger(),
                "  Orientation: yaw=%.3f [rad] (%.1f deg)", yaw, yaw * 180.0 / M_PI);
    
    // ✅ reset_ekf()関数を使用（完全リセット + 測定抑制）
    reset_ekf(x, y, yaw);
    
    // ✅ リセット後の状態を即座にパブリッシュ
    ekf_pose_.header.stamp = this->now();
    ekf_pose_.header.frame_id = map_frame_id_;
    ekf_pose_.pose.position.x = X_(0);
    ekf_pose_.pose.position.y = X_(1);
    ekf_pose_.pose.position.z = 0.0;
    ekf_pose_.pose.orientation = rpy_to_msg(0.0, 0.0, X_(2));
    
    ekf_pose_pub_->publish(ekf_pose_);
    
    RCLCPP_INFO(this->get_logger(), "✓ Initial pose published");
    RCLCPP_INFO(this->get_logger(), "✓ EKF restarted from specified position");
    RCLCPP_INFO(this->get_logger(), "========================================");
}

void EKF::ndt_pose_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
{
    ndt_pose_ = *msg;
    has_received_ndt_pose_ = true;
    
    // ✅ リセット後の抑制期間中はNDT測定更新をスキップ
    if(suppress_measurements_after_reset_) {
        if(this->now() < measurement_suppress_until_) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "NDT measurement suppressed (reset protection active)");
            return;
        } else {
            suppress_measurements_after_reset_ = false;
            RCLCPP_INFO(this->get_logger(), "✓ Measurement suppression ended - NDT updates resumed");
        }
    }
    
    if(ndt_measurement_enable_){
        measurement_update_3DoF();
    }
}


void EKF::odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
	odom_ = *msg;
    odom_.twist.twist.angular.z = 0.0; // 角速度はimuから取得するため0とする。
	has_received_odom_ = true;
	time_publish_ = msg->header.stamp;
	now_time_odom_ = msg->header.stamp;
	try{
		rclcpp::Duration duration = now_time_odom_ - last_time_odom_;
		dt_ = duration.seconds();
	}catch(std::runtime_error& ex) {
		RCLCPP_ERROR(this->get_logger(), "Exception: [%s]", ex.what());
	}
	if(is_first_odom_){
		dt_ = 0.0;
		is_first_odom_ = false;
	}
	else{
		motion_update_by_odom(dt_);
		publish_ekf_pose();
	}
	last_time_odom_ = now_time_odom_;
}

void EKF::motion_update_by_odom(double dt)
{
    Eigen::VectorXd X_prev = X_;
    Eigen::MatrixXd P_prev = P_;
    
    // Odomからの並進速度計算
    auto current_position = odom_.pose.pose.position;
    double current_v = 0.0;
    double dyaw = 0.0;
    
    Eigen::Vector3d current_odom_pose_(odom_.pose.pose.position.x, odom_.pose.pose.position.y, 0);
    
    if (!first_callback_) {
        current_v = (current_odom_pose_ - last_odom_pose_).norm() / dt;
    }
    
    last_position_ = current_position;
    first_callback_ = false;
    last_odom_pose_ = current_odom_pose_;
    
    // 制御入力
    double nu = odom_.twist.twist.linear.x;
    double omega = odom_.twist.twist.angular.z;
    
    if(std::fabs(omega) < 1e-3) omega = 1e-10;
    
    // Motion noise covariance matrix M (2x2)
    Eigen::MatrixXd M(2, 2);
    M.setZero();
    M(0,0) = std::pow(MOTION_NOISE_NN_,2)*std::fabs(nu)/dt + std::pow(MOTION_NOISE_NO_,2)*std::fabs(omega)/dt;
    M(1,1) = std::pow(MOTION_NOISE_ON_,2)*std::fabs(nu)/dt + std::pow(MOTION_NOISE_OO_,2)*std::fabs(omega)/dt;
    
    // Jacobian of motion model w.r.t. control inputs A (3x2)
    Eigen::Matrix<double,3,2> A;
    A.setZero();
    A(0,0) = (std::sin(X_(2) + omega*dt) - std::sin(X_(2)))/omega;
    A(0,1) = -nu/std::pow(omega,2)*(std::sin(X_(2) + omega*dt) - std::sin(X_(2))) + nu/omega*dt*std::cos(X_(2) + omega*dt);
    A(1,0) = (-std::cos(X_(2) + omega*dt) + std::cos(X_(2)))/omega;
    A(1,1) = -nu/std::pow(omega,2)*(-std::cos(X_(2) + omega*dt) + std::cos(X_(2))) + nu/omega*dt*std::sin(X_(2) + omega*dt);
    A(2,0) = 0.0;
    A(2,1) = dt;
    
    // Jacobian of motion model w.r.t. state G (3x3)
    Eigen::MatrixXd G(3, 3);
    G.setIdentity();
    G(0,2) = nu/omega*(std::cos(X_(2) + omega*dt) - std::cos(X_(2)));
    G(1,2) = nu/omega*(std::sin(X_(2) + omega*dt) - std::sin(X_(2)));
    
    // State transition
    if(std::fabs(omega) < 1e-2){
        X_(0) += nu*std::cos(X_(2))*dt;
        X_(1) += nu*std::sin(X_(2))*dt;
        X_(2) += omega*dt;
    }
    else{
        X_(0) += nu/omega*(std::sin(X_(2) + omega*dt) - std::sin(X_(2)));
        X_(1) += nu/omega*(-std::cos(X_(2) + omega*dt) + std::cos(X_(2)));
        X_(2) += omega*dt;
    }
    
    // 角度を正規化
    X_(2) = normalize_angle(X_(2));
    
    // Covariance update
    P_ = G * P_ * G.transpose() + A * M * A.transpose();
    
    // NaN/Infチェック
    if(std::isnan(X_(0)) || std::isnan(X_(1)) || std::isnan(X_(2)) ||
       std::isinf(X_(0)) || std::isinf(X_(1)) || std::isinf(X_(2))) {
        X_ = X_prev;
        P_ = P_prev;
        return;
    }
    
    // 変化量チェック
    double dx = X_(0) - X_prev(0);
    double dy = X_(1) - X_prev(1);
    double position_change = std::sqrt(dx * dx + dy * dy);
    double yaw_change = normalize_angle(X_(2) - X_prev(2));
    
    // 速度計算
    double velocity = position_change / dt;
    double angular_velocity = std::abs(yaw_change) / dt;
    
    // 閾値チェック
    // double max_velocity = 3.0;  // [m/s]
    // double max_angular_velocity = 2.0;  // [rad/s]
    
    if(velocity > max_odom_velocity_) {
        X_ = X_prev;
        P_ = P_prev;
        return;
    }
    
    if(angular_velocity > max_odom_angular_velocity_) {
        X_ = X_prev;
        P_ = P_prev;
        return;
    }
}

void EKF::imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
	imu_ = *msg;
	has_received_imu_ = true;
	time_publish_ = msg->header.stamp;
	now_time_imu_ = msg->header.stamp;
	try{
		rclcpp::Duration duration = now_time_imu_ - last_time_imu_;
		dt_ = duration.seconds();
	}catch(std::runtime_error& ex) {
		// ROS_ERROR("Exception: [%s]", ex.what());
		RCLCPP_ERROR(this->get_logger(), "Exception: [%s]", ex.what());
	}
	if(is_first_imu_){
		dt_ = 0.0;
		is_first_imu_ = false;
	}
	else{
		motion_update_by_imu(dt_);
		publish_ekf_pose();
	}
	last_time_imu_ = now_time_imu_;
}

void EKF::motion_update_by_imu(double dt)
{
    Eigen::VectorXd X_prev = X_;
    Eigen::MatrixXd P_prev = P_;
    
    double omega = imu_.angular_velocity.z;
    
    if(std::fabs(omega) < 1e-3) omega = 1e-10;
    
    double nu = 0.0;
    
    // Motion noise covariance matrix M (2x2)
    Eigen::MatrixXd M(2, 2);
    M.setZero();
    M(0,0) = std::pow(MOTION_NOISE_NN_,2)*std::fabs(nu)/dt + std::pow(MOTION_NOISE_NO_,2)*std::fabs(omega)/dt;
    M(1,1) = std::pow(MOTION_NOISE_ON_,2)*std::fabs(nu)/dt + std::pow(MOTION_NOISE_OO_,2)*std::fabs(omega)/dt;
    
    // Jacobian of motion model w.r.t. control inputs A (3x2)
    Eigen::Matrix<double,3,2> A;
    A.setZero();
    A(2,1) = dt;
    
    // Jacobian of motion model w.r.t. state G (3x3)
    Eigen::MatrixXd G(3, 3);
    G.setIdentity();
    
    // State transition (IMUは角度のみ更新)
    X_(2) += omega * dt;
    
    // 角度を正規化
    X_(2) = normalize_angle(X_(2));
    
    // Covariance update
    P_ = G * P_ * G.transpose() + A * M * A.transpose();
    
    // NaN/Infチェック
    if(std::isnan(X_(2)) || std::isinf(X_(2))) {
        X_ = X_prev;
        P_ = P_prev;
        return;
    }
    
    // 角速度チェック
    double yaw_change = normalize_angle(X_(2) - X_prev(2));
    double angular_velocity = std::abs(yaw_change) / dt;
    // double max_imu_angular_velocity = 2.0;  // [rad/s]
    
    if(angular_velocity > max_imu_angular_velocity_) {
        X_ = X_prev;
        P_ = P_prev;
        return;
    }
}

void EKF::measurement_update_3DoF()
{
    Eigen::VectorXd X_prev = X_;
    Eigen::MatrixXd P_prev = P_;
    
    if(check_ekf_covariance(ekf_pose_) || check_mahalanobis_distance(ekf_pose_, ndt_pose_)){
        // Z
        Eigen::VectorXd Z(X_.size());
        Z.setZero();
        Z(0) = ndt_pose_.pose.position.x;
        Z(1) = ndt_pose_.pose.position.y;
        Z(2) = calc_yaw_from_quat(ndt_pose_.pose.orientation);

        // H
        Eigen::MatrixXd H = Eigen::MatrixXd::Identity(X_.size(),X_.size());

        // I
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(X_.size(),X_.size());

        // Y
        Eigen::VectorXd Y(X_.size());
        Y.setZero();
        Y = Z - H*X_;
        Y(2) = normalize_angle(Y(2));

        // R
        Eigen::MatrixXd R(X_.size(),X_.size());
        R = SIGMA_NDT_*Eigen::MatrixXd::Identity(X_.size(),X_.size());

        // S
        Eigen::MatrixXd S(X_.size(),X_.size());
        S.setZero();
        S = H*P_*H.transpose() + R;

        // K
        Eigen::MatrixXd K(X_.size(),X_.size());
        K = P_*H.transpose()*S.inverse();

        X_ += 1.0*K*Y;
        X_(2) = normalize_angle(X_(2));
        P_ = (I - K*H)*P_;
        
        // NaN/Infチェック
        if(std::isnan(X_(0)) || std::isnan(X_(1)) || std::isnan(X_(2)) ||
           std::isinf(X_(0)) || std::isinf(X_(1)) || std::isinf(X_(2))) {
            X_ = X_prev;
            P_ = P_prev;
            return;
        }
        
        // 変化量チェック
        double dx = X_(0) - X_prev(0);
        double dy = X_(1) - X_prev(1);
        double position_change = std::sqrt(dx * dx + dy * dy);
        double yaw_change = normalize_angle(X_(2) - X_prev(2));
        double yaw_change_deg = std::abs(yaw_change) * 180.0 / M_PI;
        
        std::cout << "[NDT UPDATE CHECK]" << std::endl;
        std::cout << "  Position change: " << std::fixed << std::setprecision(3) 
                  << position_change << " m" << std::endl;
        std::cout << "  Yaw change: " << yaw_change << " rad (" 
                  << yaw_change_deg << " deg)" << std::endl;
        
        // 閾値チェック（観測更新では大きな変化も許容するが、異常値は棄却）
        // double max_ndt_position_jump = 5.0;  // [m]
        // double max_ndt_yaw_jump_deg = 90.0;   // [deg] 観測更新では大きめに
        
        if(position_change > max_ndt_position_jump_) {
            X_ = X_prev;
            P_ = P_prev;
            return;
        }
        
        if(yaw_change_deg > max_ndt_yaw_jump_deg_) {
            std::cout << "  Yaw: " << std::fixed << std::setprecision(3) 
                      << X_prev(2) << " rad → " << X_(2) << " rad" << std::endl;
            X_ = X_prev;
            P_ = P_prev;
            return;
        }
        
        std::cout << "✓ NDT update accepted" << std::endl;
    }
}

// 距離が大ならmeasurement_updateしない
// ekf_pose, ndt_pose
bool EKF::check_mahalanobis_distance(geometry_msgs::msg::PoseStamped ekf_pose, geometry_msgs::msg::PoseStamped ndt_pose)
{
	std::cout << "CHECK distance" << std::endl;

	// 共分散行列が逆行列可能かチェック
	double det = P_.determinant();
	if (std::abs(det) < 1e-12) {
		std::cout << "Covariance matrix is singular, accepting measurement" << std::endl;
		return true; // 共分散が特異の場合は測定値を受け入れ
	}

	Eigen::VectorXd ndt_eigen = Eigen::VectorXd::Zero(3);
	Eigen::VectorXd ekf_eigen = Eigen::VectorXd::Zero(3);
  	const double ndt_yaw = tf2::getYaw(ndt_pose.pose.orientation);
  	const double ekf_yaw = tf2::getYaw(ekf_pose.pose.orientation);

  	ndt_eigen << ndt_pose.pose.position.x, ndt_pose.pose.position.y, ndt_yaw;
  	// ekf_eigen << ekf_pose.pose.position.x, ekf_pose.pose.position.y, ekf_yaw;
  	ekf_eigen << X_(0), X_(1), X_(2);

	Eigen::VectorXd diff = ndt_eigen - ekf_eigen;
	diff(2) = normalize_angle(diff(2)); // ヨー角差分を-π〜πに正規化
	std::cout << "✓ Normalized angle difference: " << diff(2) << " rad" << std::endl;

	try {
		// 数値安定性のために疑似逆行列を使用
		Eigen::MatrixXd P_inv = P_.completeOrthogonalDecomposition().pseudoInverse();
		double mahalanobis_distance = std::sqrt(diff.transpose() * P_inv * diff);
		
		std::cout << "Mahalanobis distance calculated: " << mahalanobis_distance << std::endl;
		std::cout << "Threshold: " << th_mahalanobis_ << std::endl;
		
		// 閾値を超えていなかったらmeasurement_updateする
		if(mahalanobis_distance <= th_mahalanobis_){
			std::cout << "Short distance! Accepting measurement" << std::endl;
			return true;
		}
		std::cout << "Large distance! Rejecting measurement" << std::endl;
		return false;
	}
	catch(const std::exception& e) {
		std::cout << " Exception in Mahalanobis calculation: " << e.what() << std::endl;
		std::cout << " Accepting measurement due to calculation error" << std::endl;
		return true; // 計算失敗時は測定値を受け入れ
	}

	return false;
}

bool EKF::check_ekf_covariance(geometry_msgs::msg::PoseStamped ekf_pose)
{
	std::cout << "CHECK covariance" << std::endl;

	if(P_.rows() < 3 || P_.cols() < 3) {
		std::cout << " Invalid covariance matrix dimensions: " << P_.rows() << "x" << P_.cols() << std::endl;
		return true; // 行列が無効の場合は強制更新
	}
	std::cout << "Covariance matrix dimensions: " << P_.rows() << "x" << P_.cols() << std::endl;

	// ekf_poseの分散が大きければ強制measuremeent_update
	const double variance_x    = P_(0, 0);
	const double covariance_xy = P_(0, 1);
	const double variance_y    = P_(1, 1);
	const double variance_yaw  = P_(2, 2);

	if((variance_x > th_pose_covariance_) || (covariance_xy > th_pose_covariance_)
		|| (variance_y > th_pose_covariance_) || (variance_yaw > th_direction_covariance_))
	{
		std::cout << "BIG covariance... " << std::endl;
		return true; // 強制measurement update
	}
	return false;
}

void EKF::publish_ekf_pose()
{
	ekf_pose_.header.frame_id = map_frame_id_;
    ekf_pose_.header.stamp = this->get_clock()->now();
	// ekf_pose_.header.stamp = time_publish_; 
	if(std::isnan(X_(0) || std::isnan(X_(1)))){
		std::cout << "ekf_pose val NAN!!!"<< std::endl;
		return;
	}
	ekf_pose_.pose.position.x = X_(0);
	ekf_pose_.pose.position.y = X_(1);
	ekf_pose_.pose.position.z = ndt_pose_.pose.position.z;
	//ekf_pose_.pose.position.z = 0.0;
	ekf_pose_.pose.orientation = rpy_to_msg(0.0,0.0,X_(2));

	// std::cout << "EKF POSE: " << std::endl;
	// std::cout << "  X   : " << X_(0) << std::endl;
	// std::cout << "  Y   : " << X_(1) << std::endl;
	// std::cout << " YAW  : " << X_(2) << std::endl;
	// std::cout << std::endl;

	// 配列に追加
	// poses_.push_back(ekf_pose_);

	// Pathメッセージを更新
	// ekf_pose_trajectry.header.stamp = this->get_clock()->now();
	// ekf_pose_trajectry.poses = poses_;

	ekf_pose_pub_->publish(ekf_pose_);
	// path_pub_->publish(ekf_pose_trajectry);
}

void EKF::calc_rpy_from_quat(geometry_msgs::msg::Quaternion q,double& roll,double& pitch,double& yaw)
{
	tf2::Quaternion quaternion(q.x,q.y,q.z,q.w);
	tf2::Matrix3x3(quaternion).getRPY(roll,pitch,yaw);
}

geometry_msgs::msg::Quaternion EKF::rpy_to_msg(double roll,double pitch,double yaw)
{
	geometry_msgs::msg::Quaternion msg;
	tf2::Quaternion quaternion;
	quaternion.setRPY(roll,pitch,yaw);
	msg.x = quaternion.x();
	msg.y = quaternion.y();
	msg.z = quaternion.z();
	msg.w = quaternion.w();
	return msg;
}

Eigen::Matrix3d EKF::calc_rotation_matrix(Eigen::Vector3d euler_angle)
{
	Eigen::Matrix3d rotation;
	double S_r = std::sin(euler_angle(0));
	double S_p = std::sin(euler_angle(1));
	double S_y = std::sin(euler_angle(2));
	double C_r = std::cos(euler_angle(0));
	double C_p = std::cos(euler_angle(1));
	double C_y = std::cos(euler_angle(2));

	rotation <<  C_p*C_y, S_r*S_p*C_y - C_r*S_y, C_r*S_p*C_y + S_r*S_y,
		         C_p*S_y, S_r*S_p*S_y + C_r*C_y, C_r*S_p*S_y - S_r*C_y,
			    -S_p    , S_r*C_p              , C_r*C_p;
	return rotation;
}

Eigen::VectorXd EKF::measurement_function(Eigen::VectorXd x,Eigen::MatrixXd h)
{
	return h*x;
}

double EKF::normalize_angle(double angle) 
{
	 return std::atan2(std::sin(angle),std::cos(angle)); 
}

double EKF::calc_yaw_from_quat(geometry_msgs::msg::Quaternion q)
{
	double roll, pitch, yaw;
	tf2::Quaternion quaternion(q.x,q.y,q.z,q.w);
	tf2::Matrix3x3(quaternion).getRPY(roll,pitch,yaw);
	return yaw;
}

/**
 * @brief GPS FIX品質判定（RTK-FIXレベルの高精度か）
 * @param gps_pose GPSポーズメッセージ
 * @return true: 高精度FIX, false: 通常品質
 */
bool EKF::is_gps_fix_quality(const geometry_msgs::msg::PoseWithCovarianceStamped& gps_pose)
{
    // 共分散が有効かチェック
    double sigma_x = std::sqrt(gps_pose.pose.covariance[0]);
    double sigma_y = std::sqrt(gps_pose.pose.covariance[7]);
    
    // NaNチェック
    if(std::isnan(sigma_x) || std::isnan(sigma_y)) {
        return false;
    }
    
    // ゼロチェック（共分散情報が無効）
    if(sigma_x < 1e-9 || sigma_y < 1e-9) {
        return false;
    }
    
    // 水平標準偏差を計算
    double horizontal_sigma = std::sqrt(sigma_x * sigma_x + sigma_y * sigma_y);
    
    // 高精度判定（RTK-FIX想定: < 1.0m）
    bool is_high_precision = (horizontal_sigma < gps_high_confidence_threshold_);
    
    if(is_high_precision) {
        std::cout << "✓ GPS HIGH PRECISION: σ_h=" << std::fixed << std::setprecision(3) 
                  << horizontal_sigma << " m (threshold: " << gps_high_confidence_threshold_ 
                  << " m)" << std::endl;
    }
    
    return is_high_precision;
}

/**
 * @brief GPSリスポーンを実行すべきか判定
 * @param gps_pose GPSポーズメッセージ
 * @return true: リスポーン実行, false: 通常のEKF更新
 */
bool EKF::should_gps_respawn(const geometry_msgs::msg::PoseWithCovarianceStamped& gps_pose)
{
    // GPS品質チェック
    if(!is_gps_fix_quality(gps_pose)) {
        gps_high_quality_count_ = 0;
        return false;
    }
    
    gps_high_quality_count_++;
    
    std::cout << "GPS high quality count: " << gps_high_quality_count_ 
              << "/" << gps_high_quality_count_threshold_ << std::endl;
    
    if(gps_high_quality_count_ < gps_high_quality_count_threshold_) {
        return false;
    }
    
    // 距離計算
    double gps_x = gps_pose.pose.pose.position.x;
    double gps_y = gps_pose.pose.pose.position.y;
    
    double dx = gps_x - X_(0);
    double dy = gps_y - X_(1);
    double distance = std::sqrt(dx * dx + dy * dy);
    
    std::cout << "GPS-EKF distance: " << std::fixed << std::setprecision(3) 
              << distance << " m" << std::endl;
    
    // GPS高精度の場合は最小距離チェックをスキップ
    std::cout << "GPS high precision - respawn approved" << std::endl;

    // 最小距離チェック（オプション: 完全に削除してもOK）
    if(distance < gps_respawn_distance_threshold_) {
        std::cout << "Distance too small for respawn (" << std::fixed << std::setprecision(3)
                  << distance << " < " << gps_respawn_distance_threshold_ 
                  << " m)" << std::endl;
        return false;
    }
    
    // 異常値チェック（念のため残す）
    if(distance > gps_respawn_max_distance_) {
        std::cout << " Distance too large - possible GPS anomaly (" 
                  << std::fixed << std::setprecision(3) << distance << " > " 
                  << gps_respawn_max_distance_ << " m)" << std::endl;
        gps_high_quality_count_ = 0;
        return false;
    }
    
    return true;  // GPS高精度なら距離に関係なくリスポーン
}

/**
 * @brief オドメトリ基準位置を更新（リスポーン時の整合性確保）
 * @param dx X方向の移動量
 * @param dy Y方向の移動量
 */
void EKF::update_odometry_reference(double dx, double dy)
{
    // オドメトリEigen形式の基準を更新
    last_odom_eigen(0) += dx;
    last_odom_eigen(1) += dy;
    
    // オドメトリVector3d形式の基準を更新
    last_odom_pose_(0) += dx;
    last_odom_pose_(1) += dy;
    
    // オドメトリメッセージ形式の基準を更新
    if(has_received_odom_) {
        last_odom_pose.pose.pose.position.x += dx;
        last_odom_pose.pose.pose.position.y += dy;
    }
    
    std::cout << "[DEBUG] Odometry reference updated: dx=" << std::fixed 
              << std::setprecision(3) << dx << ", dy=" << dy << std::endl;
}

/**
 * @brief GPS絶対座標への段階的リスポーン
 * @param gps_x GPS絶対X座標
 * @param gps_y GPS絶対Y座標
 * @details 距離に応じて段階的に補正し、時系列情報を保持
 */
void EKF::gps_gradual_respawn(double gps_x, double gps_y)
{
    std::cout << "========================================" << std::endl;
    std::cout << "=== GPS GRADUAL RESPAWN START ===" << std::endl;
    
    gps_respawn_in_progress_ = true;
    
    // 現在のEKF状態を保存
    double prev_x = X_(0);
    double prev_y = X_(1);
    double prev_yaw = X_(2);
    
    // GPS-EKF間の差分を計算
    double dx = gps_x - prev_x;
    double dy = gps_y - prev_y;
    double distance = std::sqrt(dx * dx + dy * dy);
    
    std::cout << "GPS-EKF distance: " << std::fixed << std::setprecision(3) 
              << distance << " m" << std::endl;
    std::cout << "  EKF: (" << std::fixed << std::setprecision(3) 
              << prev_x << ", " << prev_y << ")" << std::endl;
    std::cout << "  GPS: (" << std::fixed << std::setprecision(3) 
              << gps_x << ", " << gps_y << ")" << std::endl;
    
    // 距離に応じた補正戦略を選択
    double alpha;  // GPS側の重み（0.0〜1.0）
    double suppress_duration;
    
    if(distance < 5.0) {
        // 小さい補正: GPS寄りの重み付き平均
        alpha = 0.8;  // GPS 80%
        suppress_duration = 0.0;  // 抑制なし
        std::cout << "Strategy: Small correction (alpha=" << std::fixed 
                  << std::setprecision(1) << alpha << ")" << std::endl;
    } 
    else if(distance < 15.0) {
        // 中程度の補正: バランス型
        alpha = 0.9;  // GPS 90%
        suppress_duration = measurement_suppress_duration_ * 0.3;
        std::cout << "Strategy: Medium correction (alpha=" << std::fixed 
                  << std::setprecision(1) << alpha << ")" << std::endl;
    } 
    else {
        // 大きい補正: ほぼGPS絶対座標
        alpha = 0.95;  // GPS 95%
        suppress_duration = measurement_suppress_duration_ * 0.5;
        std::cout << "Strategy: Large correction (alpha=" << std::fixed 
                  << std::setprecision(1) << alpha << ")" << std::endl;
    }
    
    // ✅ 1. 状態ベクトルを重み付き更新
    double new_x = prev_x * (1.0 - alpha) + gps_x * alpha;
    double new_y = prev_y * (1.0 - alpha) + gps_y * alpha;
    
    double actual_dx = new_x - prev_x;
    double actual_dy = new_y - prev_y;
    double actual_distance = std::sqrt(actual_dx * actual_dx + actual_dy * actual_dy);
    
    X_(0) = new_x;
    X_(1) = new_y;
    // yawは維持（GPSからは取得できないため）
    
    std::cout << "Actual correction: " << std::fixed << std::setprecision(3) 
              << actual_distance << " m" << std::endl;
    std::cout << "  New position: (" << std::fixed << std::setprecision(3) 
              << new_x << ", " << new_y << ")" << std::endl;
    
    // ✅ 2. オドメトリ基準位置を同じだけシフト
    update_odometry_reference(actual_dx, actual_dy);
    
    // ✅ 3. 共分散行列を適切に調整
    // GPS共分散を取得
    double gps_var_x = gps_pose_.pose.covariance[0];
    double gps_var_y = gps_pose_.pose.covariance[7];
    
    // 位置の共分散をGPSの精度に基づいて設定
    if(gps_var_x > 1e-9 && gps_var_y > 1e-9) {
        // GPS共分散が有効な場合
        double gps_weight = alpha;
        double ekf_weight = 1.0 - alpha;
        
        // 重み付き共分散（簡易的な融合）
        P_(0, 0) = ekf_weight * P_(0, 0) + gps_weight * gps_var_x * 4.0;
        P_(1, 1) = ekf_weight * P_(1, 1) + gps_weight * gps_var_y * 4.0;
    } else {
        // GPS共分散が無効な場合は距離ベース
        double position_uncertainty = actual_distance * 0.1;  // 移動量の10%
        P_(0, 0) = std::max(P_(0, 0), position_uncertainty * position_uncertainty);
        P_(1, 1) = std::max(P_(1, 1), position_uncertainty * position_uncertainty);
    }
    
    // yaw共分散は小さく増加
    P_(2, 2) = P_(2, 2) * 1.1;
    
    // 位置とyawの相関をリセット（大きく補正した場合）
    if(distance > 10.0) {
        P_(0, 2) = 0.0;
        P_(2, 0) = 0.0;
        P_(1, 2) = 0.0;
        P_(2, 1) = 0.0;
    }
    
    std::cout << "Covariance updated:" << std::endl;
    std::cout << "  P(0,0)=" << std::fixed << std::setprecision(4) << P_(0,0) 
              << ", P(1,1)=" << P_(1,1) << ", P(2,2)=" << P_(2,2) << std::endl;
    
    // ✅ 4. 大きく補正した場合はNDT情報をクリア
    if(distance > 20.0) {
        has_received_ndt_pose_ = false;
        ndt_pose_ = geometry_msgs::msg::PoseStamped();
        std::cout << "Large correction - NDT pose cleared (will re-acquire)" << std::endl;
    }
    
    // ✅ 5. 測定更新の一時的抑制
    if(suppress_duration > 0.0) {
        suppress_measurements_after_reset_ = true;
        measurement_suppress_until_ = this->now() + 
            rclcpp::Duration::from_seconds(suppress_duration);
        std::cout << "NDT/GPS measurements suppressed for " << std::fixed 
                  << std::setprecision(1) << suppress_duration << " sec" << std::endl;
    }
    
    // カウンタリセット
    gps_high_quality_count_ = 0;
    gps_respawn_in_progress_ = false;
    
    std::cout << "=== GPS GRADUAL RESPAWN COMPLETE ===" << std::endl;
    std::cout << "========================================" << std::endl;
}

void EKF::gps_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg)
{
    gps_pose_ = *msg;
    has_received_gps_ = true;
    static bool first_callback_ = true;

    // 測定抑制中かチェック
    rclcpp::Time current_time = this->now();
    if(measurement_suppress_until_.nanoseconds() > 0) {
        if(current_time < measurement_suppress_until_) {
            std::cout << "[DEBUG] GPS measurement suppressed (respawn in progress)" << std::endl;
            has_received_gps_ = false;
            return;
        } else {
            // 抑制期間が終了
            measurement_suppress_until_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
            std::cout << "✓ Measurement suppression ended - resuming normal operation" << std::endl;
        }
    }

    // 初回GPS受信時に自動初期化
    if (first_callback_ && is_gps_fix_quality(*msg)) {
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;
        double z = msg->pose.pose.position.z;
        
        RCLCPP_INFO(this->get_logger(), 
            "Auto-initializing EKF from GPS: x=%.3f, y=%.3f, z=%.3f", x, y, z);
        
        initialize(x, y, z, INIT_ROLL_, INIT_PITCH_, INIT_YAW_);
        first_callback_ = false;
    }
    
    
    // GPSリスポーン機能が無効の場合は通常のGPS測定更新のみ
    if(!gps_respawn_enable_) {
        static bool once = false;
        if(!once) {
            std::cout << "[DEBUG] GPS respawn disabled - using standard GPS measurement update only" 
                      << std::endl;
            once = true;
        }
        
        // 通常のGPS測定更新
        if(gps_measurement_enable_) {
            measurement_update_gps();
        }
        return;
    }
    
    // GPSリスポーン判定
    if(should_gps_respawn(gps_pose_)) {
        // 高精度GPS時のリスポーン実行
        double gps_x = gps_pose_.pose.pose.position.x;
        double gps_y = gps_pose_.pose.pose.position.y;
        
        gps_gradual_respawn(gps_x, gps_y);
    } 
    else {
        // 通常のGPS測定更新
        if(gps_measurement_enable_) {
            measurement_update_gps();
        }
    }
}

void EKF::measurement_update_gps()
{
    if(!has_received_gps_) return;
    
    std::cout << "[DEBUG] GPS standard measurement update" << std::endl;
    
    // Extract GPS position
    double gps_x = gps_pose_.pose.pose.position.x;
    double gps_y = gps_pose_.pose.pose.position.y;
    
    // Mahalanobis distance check (optional - for outlier rejection)
    Eigen::VectorXd gps_pos(2);
    gps_pos << gps_x, gps_y;
    
    Eigen::VectorXd ekf_pos(2);
    ekf_pos << X_(0), X_(1);
    
    Eigen::VectorXd diff = gps_pos - ekf_pos;
    
    // Extract position covariance (2x2)
    Eigen::MatrixXd P_pos = P_.block<2,2>(0,0);
    double det = P_pos.determinant();
    
    if(std::abs(det) > 1e-12) {
        try {
            Eigen::MatrixXd P_pos_inv = P_pos.inverse();
            double mahalanobis_dist = std::sqrt(diff.transpose() * P_pos_inv * diff);
            
            std::cout << "[DEBUG] GPS Mahalanobis distance: " << std::fixed 
                      << std::setprecision(3) << mahalanobis_dist << std::endl;
            
            // Reject if distance is too large (threshold can be adjusted)
            if(mahalanobis_dist > 10.0) {
                std::cout << "GPS measurement rejected (large Mahalanobis distance: " 
                          << std::fixed << std::setprecision(3) << mahalanobis_dist 
                          << ")" << std::endl;
                has_received_gps_ = false;
                return;
            }
        } catch(const std::exception& e) {
            std::cout << "GPS Mahalanobis calculation error: " << e.what() << std::endl;
        }
    }
    
    // Observation vector Z (x, y only)
    Eigen::VectorXd Z(2);
    Z << gps_x, gps_y;
    
    // Observation matrix H (observe x and y only)
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2, X_.size());
    H(0, 0) = 1.0;  // x
    H(1, 1) = 1.0;  // y
    
    // Identity matrix
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(X_.size(), X_.size());
    
    // Innovation Y
    Eigen::VectorXd Y = Z - H * X_;
    
    // Observation noise covariance R (use GPS covariance)
    Eigen::MatrixXd R(2, 2);
    R << gps_pose_.pose.covariance[0], gps_pose_.pose.covariance[1],
         gps_pose_.pose.covariance[6], gps_pose_.pose.covariance[7];
    
    // Add minimum variance if GPS covariance is too small
    if(R(0,0) < 1e-6) R(0,0) = SIGMA_GPS_ * SIGMA_GPS_;
    if(R(1,1) < 1e-6) R(1,1) = SIGMA_GPS_ * SIGMA_GPS_;
    
    // Innovation covariance S
    Eigen::MatrixXd S = H * P_ * H.transpose() + R;
    
    // Kalman gain K
    Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();
    
    // State update
    X_ += K * Y;
    
    // Covariance update
    P_ = (I - K * H) * P_;
    
    std::cout << "[DEBUG] GPS update: dx=" << std::fixed << std::setprecision(3) 
              << (K * Y)(0) << ", dy=" << (K * Y)(1) << std::endl;
    std::cout << "[DEBUG] Position: x=" << std::fixed << std::setprecision(3) 
              << X_(0) << ", y=" << X_(1) << std::endl;
    
    has_received_gps_ = false;
}

void EKF::process()
{
}

int main(int argc,char** argv)
{
	std::cout << "---ekf---" << std::endl;
    rclcpp::init(argc, argv); // ノードの初期化
    auto node = std::make_shared<EKF>();
    rclcpp::spin(node);
    rclcpp::shutdown();
	return 0;
}
