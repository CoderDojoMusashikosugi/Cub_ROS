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
	// last_time_ = this->get_clock()->now();
	ekf_pose_trajectry.header.frame_id = "map"; 
	std::cout << "\n[THRESHOLD PARAMETERS]" << std::endl;
    std::cout << "  TH_MAHALANOBIS          : " << std::fixed << th_mahalanobis_ << std::endl;
    std::cout << "  TH_COVARIANCE           : " << std::fixed  << th_covariance_ << std::endl;
    std::cout << "  TH_POSE_COVARIANCE      : " << std::fixed << th_pose_covariance_ << std::endl;
    std::cout << "  TH_DIRECTION_COVARIANCE : " << std::fixed << th_direction_covariance_ << std::endl;
	std::cout << "  SIGMA_GPS               : " << std::fixed << SIGMA_GPS_ << std::endl;
	std::cout << "  GPS_MEASUREMENT :                : " << std::fixed << gps_measurement_enable_ << std::endl;
	std::cout << "  NDT_MEASUREMENT :                : " << std::fixed << ndt_measurement_enable_ << std::endl;
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
 * @brief RVizの2D Pose Estimateからの初期位置設定コールバック
 * @param msg 初期位置メッセージ
 */
void EKF::initialpose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "========================================");
    RCLCPP_INFO(this->get_logger(), "Received initial pose from RViz 2D Pose Estimate");
    
    // クォータニオンからyawのみを計算
    double yaw = calc_yaw_from_quat(msg->pose.pose.orientation);
    
    // 受信した位置情報を取得 (x, yのみ使用、zは無視)
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    
    RCLCPP_INFO(this->get_logger(), 
                "Resetting EKF state to:");
    RCLCPP_INFO(this->get_logger(),
                "  Position: x=%.3f, y=%.3f [m]", x, y);
    RCLCPP_INFO(this->get_logger(),
                "  Orientation: yaw=%.3f [rad] (%.1f deg)", yaw, yaw * 180.0 / M_PI);
    
    // ✅ 3DoF状態ベクトルの更新 (x, y, yaw)
    X_(0) = x;
    X_(1) = y;
    X_(2) = yaw;
    
    // 共分散行列を初期値にリセット
    P_ = Eigen::MatrixXd::Identity(STATE_SIZE_, STATE_SIZE_) * INIT_SIGMA_;
    
    RCLCPP_INFO(this->get_logger(), "Reset covariance matrix (3x3)");
    
    // センサー受信フラグのリセット
    is_first_odom_ = true;
    is_first_imu_ = true;
    
    // オドメトリ関連の変数をリセット
    last_odom_eigen = Eigen::Vector3d::Zero();
    last_odom_pose_ = Eigen::Vector3d::Zero();
    last_odom_yaw_ = 0.0;
    
    // タイムスタンプの更新
    time_publish_ = msg->header.stamp;
    last_time_odom_ = msg->header.stamp;
    last_time_imu_ = msg->header.stamp;
    
    // 更新後の状態を即座にパブリッシュ
    ekf_pose_.header.stamp = msg->header.stamp;
    ekf_pose_.header.frame_id = map_frame_id_;
    ekf_pose_.pose.position.x = X_(0);
    ekf_pose_.pose.position.y = X_(1);
    ekf_pose_.pose.position.z = 0.0;  // 3DoFなのでz=0
    ekf_pose_.pose.orientation = rpy_to_msg(0.0, 0.0, X_(2));  // roll=0, pitch=0, yaw=X_(2)
    
    ekf_pose_pub_->publish(ekf_pose_);
    
    RCLCPP_INFO(this->get_logger(), "EKF state reset completed");
    RCLCPP_INFO(this->get_logger(), "========================================");
}

void EKF::ndt_pose_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
{
	ndt_pose_ = *msg;
	has_received_ndt_pose_ = true;
	if(ndt_measurement_enable_){
		measurement_update_3DoF();
	}
}

void EKF::odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
	odom_ = *msg;
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
		std::cout << "odom motioin_update" << std::endl;
		motion_update_by_odom(dt_);
		publish_ekf_pose();
	}
	last_time_odom_ = now_time_odom_;
}

void EKF::motion_update_by_odom(double dt)
{
    // Odomからの並進速度計算
    auto current_position = odom_.pose.pose.position;
    double current_v = 0.0;
	double  dyaw = 0.0;
	// double omega = 0.0;
    
    Eigen::Vector3d current_odom_pose_(odom_.pose.pose.position.x, odom_.pose.pose.position.y, 0);
	// 現在の姿勢角を取得
    // double current_yaw = calc_yaw_from_quat(odom_.pose.pose.orientation);
    
    if (!first_callback_) {
        // 位置の変化量から速度を計算
        current_v = (current_odom_pose_ - last_odom_pose_).norm() / dt;
		// 姿勢角の変化量から角速度を計算
        // dyaw = normalize_angle(current_yaw - last_odom_yaw_);
        // omega = dyaw / dt;
    }
    
    // 現在の位置を保存
    last_position_ = current_position;
    first_callback_ = false;
    last_odom_pose_ = current_odom_pose_;
	// last_odom_yaw_ = current_yaw;
    
    // 制御入力
    // double nu = current_v;
	double nu = odom_.twist.twist.linear.x; // 使えるならこれを使うべき
    double omega = odom_.twist.twist.angular.z; // Odomから角速度を取得
    
    // 微小角速度の場合は数値安定性のため小さな値に設定
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
        // 角速度が小さい場合は線形近似を使用
        X_(0) += nu*std::cos(X_(2))*dt;
        X_(1) += nu*std::sin(X_(2))*dt;
        X_(2) += omega*dt;
    }
    else{
        // 通常の非線形モデル
        X_(0) += nu/omega*(std::sin(X_(2) + omega*dt) - std::sin(X_(2)));
        X_(1) += nu/omega*(-std::cos(X_(2) + omega*dt) + std::cos(X_(2)));
        X_(2) += omega*dt;
    }
    
    // Covariance update
    P_ = G * P_ * G.transpose() + A * M * A.transpose();
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
		std::cout << "imu motioin_update" << std::endl;
		motion_update_by_imu(dt_);
		publish_ekf_pose();
	}
	last_time_imu_ = now_time_imu_;
}
void EKF::motion_update_by_imu(double dt)
{
    // IMUからの角速度を取得
    double omega = imu_.angular_velocity.z;
    
    // 微小角速度の場合は数値安定性のため小さな値に設定
    if(std::fabs(omega) < 1e-3) omega = 1e-10;
    
    // 並進速度は0と仮定（IMUからは取得できないため）
    double nu = 0.0;
    
    // Motion noise covariance matrix M (2x2)
    Eigen::MatrixXd M(2, 2);
    M.setZero();
    M(0,0) = std::pow(MOTION_NOISE_NN_,2)*std::fabs(nu)/dt + std::pow(MOTION_NOISE_NO_,2)*std::fabs(omega)/dt;
    M(1,1) = std::pow(MOTION_NOISE_ON_,2)*std::fabs(nu)/dt + std::pow(MOTION_NOISE_OO_,2)*std::fabs(omega)/dt;
    
    // Jacobian of motion model w.r.t. control inputs A (3x2)
    Eigen::Matrix<double,3,2> A;
    A.setZero();
    // IMUは角速度のみなので、角速度に関する項のみ設定
    A(2,1) = dt; // dyaw/domega = dt
    
    // Jacobian of motion model w.r.t. state G (3x3)
    Eigen::MatrixXd G(3, 3);
    G.setIdentity();
    // IMUは並進速度がないため、yawに関するJacobianは0
    
    // State transition (IMUは角度のみ更新)
    X_(2) += omega * dt;
    
    // Covariance update
    P_ = G * P_ * G.transpose() + A * M * A.transpose();
}

void EKF::measurement_update_3DoF()
{
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

	// // ekf_poseとndt_poseのマハラノビス距離を算出
  	// double mahalanobis_distance =std::sqrt((ndt_eigen - ekf_eigen).transpose() * P_.inverse() * (ndt_eigen - ekf_eigen));
	// std::cout << "done calc mahalanobis distance!" << std::endl;
	// // 閾値を超えていなかったらmeasurement_updateする
	// if(mahalanobis_distance <= th_mahalanobis_){
	// 	std::cout << "short distance!" << std::endl;
	// 	return true;
	// }
	// std::cout << "large distance! reject" << std::endl;
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
	ekf_pose_.header.stamp = time_publish_; 
	if(std::isnan(X_(0) || std::isnan(X_(1)))){
		std::cout << "ekf_pose val NAN!!!"<< std::endl;
		return;
	}
	ekf_pose_.pose.position.x = X_(0);
	ekf_pose_.pose.position.y = X_(1);
	ekf_pose_.pose.position.z = ndt_pose_.pose.position.z;
	//ekf_pose_.pose.position.z = 0.0;
	ekf_pose_.pose.orientation = rpy_to_msg(0.0,0.0,X_(2));

	std::cout << "EKF POSE: " << std::endl;
	std::cout << "  X   : " << X_(0) << std::endl;
	std::cout << "  Y   : " << X_(1) << std::endl;
	std::cout << " YAW  : " << X_(2) << std::endl;
	std::cout << std::endl;

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

void EKF::gps_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg)
{
    gps_pose_ = *msg;
    has_received_gps_ = true;
	if(gps_measurement_enable_){
		measurement_update_gps();
	}
}

void EKF::measurement_update_gps()
{
    if(!has_received_gps_) return;
    
    std::cout << "GPS measurement update" << std::endl;
    
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
            
            std::cout << "GPS Mahalanobis distance: " << mahalanobis_dist << std::endl;
            
            // Reject if distance is too large (threshold can be adjusted)
            if(mahalanobis_dist > 10.0) {
                std::cout << "GPS measurement rejected (large Mahalanobis distance)" << std::endl;
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
    
    std::cout << "GPS update applied: dx=" << (K * Y)(0) 
              << ", dy=" << (K * Y)(1) << std::endl;
    std::cout << "Updated position: x=" << X_(0) << ", y=" << X_(1) << std::endl;
    
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
	// now_time_ = ros::Time::now();
	// bool is_first_ = true;
	// rclcpp::Rate rate(node->hz_);
	while(rclcpp::ok()){
		// node->process();
		// last_time_ = now_time_;
		rclcpp::spin_some(node);
		// rate.sleep();
	}
	return 0;
}