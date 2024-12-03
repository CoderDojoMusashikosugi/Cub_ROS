#include "ekf/ekf.h"

EKF::EKF() : Node("EKF")
{
	this->declare_parameter<std::string>("ndt_pose_topic_name", "/ndt_pose");
    this->declare_parameter<std::string>("imu_topic_name", "/bno055/imu");
    this->declare_parameter<std::string>("odom_topic_name", "/odom");
    this->declare_parameter<std::string>("ekf_pose_topic_name", "/ekf_pose");
    this->declare_parameter<std::string>("map_frame_id", "map");
    this->declare_parameter<std::string>("odom_frame_id", "odom");
    this->declare_parameter<std::string>("base_link_frame_id", "base_link");
    this->declare_parameter<bool>("is_3DoF", true);
    this->declare_parameter<bool>("is_odom_tf", false);
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

    // Retrieve the parameters
    this->get_parameter("ndt_pose_topic_name", ndt_pose_topic_name_);
    this->get_parameter("imu_topic_name", imu_topic_name_);
    this->get_parameter("odom_topic_name", odom_topic_name_);
    this->get_parameter("ekf_pose_topic_name", ekf_pose_topic_name_);
    this->get_parameter("map_frame_id", map_frame_id_);
    this->get_parameter("odom_frame_id", odom_frame_id_);
    this->get_parameter("base_link_frame_id", base_link_frame_id_);
    this->get_parameter("is_3DoF", is_3DoF_);
    this->get_parameter("is_odom_tf", is_odom_tf_);

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


    ndt_pose_sub_  = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/ndt_pose", rclcpp::QoS(1).reliable(),
        std::bind(&EKF::ndt_pose_callback, this, std::placeholders::_1));
    imu_sub_  = this->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_name_, rclcpp::QoS(1).reliable(),
        std::bind(&EKF::imu_callback, this, std::placeholders::_1));
    odom_sub_  = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", rclcpp::QoS(1).reliable(),
        std::bind(&EKF::odom_callback, this, std::placeholders::_1));

    ekf_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/ekf_pose", rclcpp::QoS(1).reliable());


	auto clock = this->get_clock();

	initialize(INIT_X_, INIT_Y_, INIT_Z_, INIT_ROLL_, INIT_PITCH_, INIT_YAW_);
	is_measurement_.data = false;
	last_time_ = this->get_clock()->now();
}

EKF::~EKF() {}

void EKF::ndt_pose_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
{
	ndt_pose_ = *msg;
	has_received_ndt_pose_ = true;
}

void EKF::odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
	odom_ = *msg;

	if(is_odom_tf_){
		geometry_msgs::msg::TransformStamped transform;
		transform.header = msg->header;
		transform.header.frame_id = odom_frame_id_;
    	transform.child_frame_id = base_link_frame_id_;

    	transform.transform.translation.x = msg->pose.pose.position.x;
    	transform.transform.translation.y = msg->pose.pose.position.y;
    	transform.transform.translation.z = msg->pose.pose.position.z;
    	transform.transform.rotation = msg->pose.pose.orientation;

		broadcaster_->sendTransform(transform);
	}

	has_received_odom_ = true;
}

void EKF::imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
	imu_ = *msg;
	has_received_imu_ = true;
}

void EKF::measurement_callback(const std_msgs::msg::Bool::ConstSharedPtr msg)
{
	is_measurement_ = *msg;
}


void EKF::initialize(double x,double y,double z,double roll,double pitch,double yaw)
{
	if(is_3DoF_) STATE_SIZE_ = 3;
	else STATE_SIZE_ = 6;

	X_.setZero(STATE_SIZE_);
	P_.setZero(STATE_SIZE_, STATE_SIZE_);
	P_ = INIT_SIGMA_*Eigen::MatrixXd::Identity(STATE_SIZE_, STATE_SIZE_);
	set_pose(x,y,z,roll,pitch,yaw);
}

void EKF::set_pose(double x,double y,double z,double roll,double pitch,double yaw)
{
	if(is_3DoF_){
		if(X_.size() != STATE_SIZE_){
			std::cout << "No matching STATE SIZE" << std::endl;
			return;
		}
		X_(0) = x;
		X_(1) = y;
		X_(2) = yaw;
	}
	else{
		if(X_.size() != STATE_SIZE_){
			std::cout << "No matching STATE SIZE" << std::endl;
		}
		X_(0) = x;
		X_(1) = y;
		X_(2) = z;
		X_(3) = roll;
		X_(4) = pitch;
		X_(5) = yaw;
	}
}

void EKF::calc_rpy_from_quat(geometry_msgs::msg::Quaternion q,double& roll,double& pitch,double& yaw)
{
	tf2::Quaternion quaternion(q.x,q.y,q.z,q.w);
	tf2::Matrix3x3(quaternion).getRPY(roll,pitch,yaw);
}

void EKF::motion_update_3DoF(double dt)
{
	// twist情報の作成
	auto current_position = odom_.pose.pose.position;
	auto current_time = this->get_clock()->now();
	geometry_msgs::msg::Twist twist_;
	double current_v;

	Eigen::Vector3d current_odom_pose_(odom_.pose.pose.position.x, odom_.pose.pose.position.y, 0);

	if (!first_callback_) {
		// 位置の変化量を計算
		double dx = current_position.x - last_position_.x;
		double dy = current_position.y - last_position_.y;
		double dz = current_position.z - last_position_.z;
		// 経過時間を計算（秒単位）
		// double dt = (current_time - last_time_).seconds();

		// double distance = std::sqrt(dx * dx + dy * dy + dz * dz);	
		// if (distance > 1.0) {  // 適切な閾値に設定
		// 	RCLCPP_WARN(this->get_logger(), "Position change is too large, skipping update");
		// 	return;
		// }
		current_v = (current_odom_pose_ - last_odom_pose_).norm() / dt;

		// if (dt > 0) {
		// 	// 速度を計算
		// 	twist_.linear.x = dx / dt;
		// 	twist_.linear.y = dy / dt;
		// 	twist_.linear.z = dz / dt;
		// }
	}
	// 現在の位置と時刻を保存
	last_position_ = current_position;
	last_time_ = current_time;
	first_callback_ = false;

	last_odom_pose_ = current_odom_pose_;
	
	double nu = current_v;
	// double nu = twist_.linear.x;
	// double nu = odom_.twist.twist.linear.x;
	double omega = imu_.angular_velocity.z; // ここは大丈夫そう odomに切り替えてもバックしたので
	// double omega = odom_.twist.twist.angular.z;
	// double omega = get_yaw(odom_.pose.pose.orientation);


	if(std::fabs(omega) < 1e-3) omega = 1e-10;

	// M
	Eigen::MatrixXd M(X_.size() - 1,X_.size() - 1);
	M.setZero();
	M(0,0) = SIGMA_ODOM_*SIGMA_ODOM_;
	M(1,1) = SIGMA_IMU_*SIGMA_IMU_;
	//M(0,0) = std::pow(MOTION_NOISE_NN_,2)*std::fabs(nu)/dt + std::pow(MOTION_NOISE_NO_,2)*std::fabs(omega)/dt;
	//M(1,1) = std::pow(MOTION_NOISE_ON_,2)*std::fabs(nu)/dt + std::pow(MOTION_NOISE_OO_,2)*std::fabs(omega)/dt;

	// A
	Eigen::Matrix<double,3,2> A;
	A.setZero();
	A(0,0) = (std::sin(X_(2) + omega*dt) - std::sin(X_(2)))/omega;
	A(0,1) = -nu/std::pow(omega,2)*(std::sin(X_(2) + omega*dt) - std::sin(X_(2))) + nu/omega*dt*std::cos(X_(2) + omega*dt);
	A(1,0) = (-std::cos(X_(2) + omega*dt) + std::cos(X_(2)))/omega;
	A(1,1) = -nu/std::pow(omega,2)*(-std::cos(X_(2) + omega*dt) + std::cos(X_(2))) + nu/omega*dt*std::sin(X_(2) + omega*dt);
	A(2,0) = 0.0;
	A(2,1) = dt;

	// G
	Eigen::MatrixXd G(X_.size(),X_.size());
	G.setIdentity();
	G(0,2) = nu/omega*(std::cos(X_(2) + omega*dt) - std::cos(X_(2)));
	G(1,2) = nu/omega*(std::sin(X_(2) + omega*dt) - std::sin(X_(2)));

	// state transition
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
	
	/*
	X_(0) += nu*std::cos(X_(2))*dt;
	X_(1) += nu*std::sin(X_(2))*dt;
	X_(2) += omega*dt;
	*/

	P_ = G*P_*G.transpose() + A*M*A.transpose();
}

// double EKF::get_yaw(geometry_msgs::Quaternion q)
// {
//     double roll , pitch , yaw;
//     tf::Quaternion quat(q.x, q.y, q.z, q.w);
//     tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
//     return yaw;
// }



void EKF::motion_update(double dt)
{
	if(is_3DoF_) motion_update_3DoF(dt);
	else motion_update_6DoF(dt);
}

void EKF::measurement_update_3DoF()
{
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
	P_ = (I - K*H)*P_;
}


void EKF::measurement_update()
{
	if(is_3DoF_) measurement_update_3DoF();
	else measurement_update_6DoF();
}


void EKF::publish_ekf_pose()
{
	ekf_pose_.header.frame_id = map_frame_id_;
	ekf_pose_.pose.position.x = X_(0);
	ekf_pose_.pose.position.y = X_(1);
	if(is_3DoF_){
		ekf_pose_.pose.position.z = ndt_pose_.pose.position.z;
		//ekf_pose_.pose.position.z = 0.0;
		ekf_pose_.pose.orientation = rpy_to_msg(0.0,0.0,X_(2));


		std::cout << "EKF POSE: " << std::endl;
		std::cout << "  X   : " << X_(0) << std::endl;
		std::cout << "  Y   : " << X_(1) << std::endl;
		std::cout << " YAW  : " << X_(2) << std::endl;
		std::cout << std::endl;

	}
	else{
		ekf_pose_.pose.position.z = X_(2);
		ekf_pose_.pose.orientation = rpy_to_msg(X_(3),X_(4),X_(5));

		std::cout << "EKF POSE: " << std::endl;
		std::cout << "  X   : " << X_(0) << std::endl;
		std::cout << "  Y   : " << X_(1) << std::endl;
		std::cout << "  Z   : " << X_(2) << std::endl;
		std::cout << " ROLL : " << X_(3) << std::endl;
		std::cout << "PITCH : " << X_(4) << std::endl;
		std::cout << " YAW  : " << X_(5) << std::endl;
		std::cout << std::endl;
	}

	// ekf_pose_pub_.publish(ekf_pose_);
	ekf_pose_pub_->publish(ekf_pose_);
}

// void EKF::publish_tf()
// {
// 	try{
// 		tf2::Quaternion q;
// 		tf2::Transform map_transform;
// 		if(is_3DoF_){
// 			q.setRPY(0.0,0.0,X_(2));
// 			//tf2::Transform transform(q,tf2::Vector3(X_(0),X_(1),0.0));
// 			tf2::Transform transform(q,tf2::Vector3(X_(0),X_(1),ndt_pose_.pose.position.z));
// 			map_transform = transform;
// 		}
// 		else{
// 			q.setRPY(X_(3),X_(4),X_(5));
// 			tf2::Transform transform(q,tf2::Vector3(X_(0),X_(1),X_(2)));
// 			map_transform = transform;
// 		}
// 		geometry_msgs::msg::PoseStamped tf_stamped;
// 		tf_stamped.header.frame_id = base_link_frame_id_;
// 		tf_stamped.header.stamp = odom_.header.stamp;
// 		tf2::toMsg(map_transform.inverse(),tf_stamped.pose);
// 		geometry_msgs::msg::PoseStamped odom_to_map;
// 		buffer_->transform(tf_stamped,odom_to_map,odom_frame_id_);

// 		tf2::Transform latest_tf;
// 		tf2::convert(odom_to_map.pose,latest_tf);
// 		geometry_msgs::msg::TransformStamped tmp_tf_stamped;
// 		// geometry_msgs::TransformStamped tmp_tf_stamped;
// 		tmp_tf_stamped.header.stamp = odom_.header.stamp;
// 		tmp_tf_stamped.header.frame_id = map_frame_id_;
// 		tmp_tf_stamped.child_frame_id = odom_frame_id_;
// 		tf2::convert(latest_tf.inverse(),tmp_tf_stamped.transform);
// 		broadcaster_->sendTransform(tmp_tf_stamped);
// 	}
// 	catch(tf2::TransformException& ex){
// 		// ROS_WARN("%s", ex.what());
// 		return;
// 	}

// }

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

double EKF::normalize_angle(double angle) { return std::atan2(std::sin(angle),std::cos(angle)); }

double EKF::calc_yaw_from_quat(geometry_msgs::msg::Quaternion q)
{
	double roll, pitch, yaw;
	tf2::Quaternion quaternion(q.x,q.y,q.z,q.w);
	tf2::Matrix3x3(quaternion).getRPY(roll,pitch,yaw);

	return yaw;
}

void EKF::process()
{
	if(has_received_imu_ && has_received_odom_){
		// now_time_ = ros::Time::now();
		if(is_first_){
			dt = 1.0/(double)10;
			is_first_ = false;
		}
		// else dt = now_time_.toSec() - last_time_.toSec();
		motion_update(1.0/(double)10);
		if(has_received_ndt_pose_) measurement_update();
		//if(has_received_ndt_pose_ && !is_measurement_.data) measurement_update();
		publish_ekf_pose();
		// publish_tf();
		has_received_ndt_pose_ = false;
	}
}
// publish_tf()は一旦放置
int main(int argc,char** argv)
{
	std::cout << "---ekf---" << std::endl;
    rclcpp::init(argc, argv); // ノードの初期化
    auto node = std::make_shared<EKF>();
	// now_time_ = ros::Time::now();
	// bool is_first_ = true;
	rclcpp::Rate rate(10);
	while(rclcpp::ok()){
		node->process();
		// last_time_ = now_time_;
		rclcpp::spin_some(node);
		rate.sleep();
	}
	return 0;
}

void EKF::measurement_update_6DoF()
{
	std::cout << "measurement" << std::endl;

	// Z
	Eigen::VectorXd Z(X_.size());
	Z(0) = ndt_pose_.pose.position.x;
	Z(1) = ndt_pose_.pose.position.y;
	Z(2) = ndt_pose_.pose.position.z;
	calc_rpy_from_quat(ndt_pose_.pose.orientation,Z(3),Z(4),Z(5));

	// H
	Eigen::MatrixXd H = Eigen::MatrixXd::Identity(X_.size(),X_.size());

	// I
	Eigen::MatrixXd I = Eigen::MatrixXd::Identity(X_.size(),X_.size());

	// Y
	Eigen::VectorXd Y = Z - measurement_function(X_,H);

	// R
	Eigen::MatrixXd R = SIGMA_NDT_*Eigen::MatrixXd::Identity(X_.size(),X_.size());

	// S
	Eigen::MatrixXd S = H*P_*H.transpose() + R;

	// K
	Eigen::MatrixXd K = P_*H.transpose()*S.inverse();

	X_ += K*Y;
	P_ = (I - K*H)*P_;

	for(int i = 3; i < 6; i++) X_(i) = normalize_angle(X_(i));
}

void EKF::motion_update_6DoF(double dt)
{
	double roll = X_(3) ;
	double pitch = X_(4);
	double yaw = X_(5) ;

	double delta_yaw = imu_.angular_velocity.z*dt;
	double delta_roll = imu_.angular_velocity.x*dt;
	double delta_pitch = imu_.angular_velocity.y*dt;

	if(delta_yaw < 1e-3) delta_yaw = 0.0;
	if(delta_roll < 1e-3) delta_roll = 0.0;
	if(delta_pitch < 1e-3) delta_pitch = 0.0;

	Eigen::Vector3d delta_position = { odom_.twist.twist.linear.x*dt, 0.0, 0.0 };
	Eigen::Vector3d delta_rotation = { delta_roll, delta_pitch, delta_yaw };
	Eigen::Matrix3d rotation_rpy;
	rotation_rpy << 1, std::sin(roll)*std::tan(pitch), std::cos(roll)*std::tan(pitch),
	                0, std::cos(roll)                , -std::sin(roll)               ,
					0, std::sin(roll)/std::cos(pitch), std::cos(roll)/std::cos(pitch);

	Eigen::VectorXd F(X_.size());
	F.segment(0,3) = X_.segment(0,3) + calc_rotation_matrix(X_.segment(3,3))*delta_position;
	F.segment(3,3) = X_.segment(3,3) + rotation_rpy*delta_rotation;
	for(int i = 3; i < 6; i++) F(i) = normalize_angle(F(i));

	Eigen::MatrixXd G(X_.size(),X_.size());
	G.block(0,0,3,3) = Eigen::Matrix3d::Identity();
	G.block(3,3,3,3) = Eigen::Matrix3d::Identity();
	G.block(3,0,3,3) = Eigen::Matrix3d::Zero();
	G.block(0,3,3,3) = Eigen::Matrix3d::Zero();

	G(3,3) = 1.0 + std::cos(roll)*std::tan(pitch)*delta_pitch - std::sin(roll)*std::tan(pitch)*delta_yaw;
	G(3,4) = std::sin(roll)/std::cos(pitch)/std::cos(pitch)*delta_pitch + std::cos(roll)/std::cos(pitch)/std::cos(pitch)*delta_yaw;
	G(3,5) = 0.0;
	G(4,3) = -std::sin(roll)*delta_pitch - std::cos(roll)*delta_yaw;
	G(4,4) = 1.0;
	G(4,5) = 0.0;
	G(5,3) = std::cos(roll)/std::cos(pitch)*delta_pitch - std::sin(roll)/std::cos(pitch)*delta_yaw;
	G(5,4) = std::sin(roll)*std::sin(pitch)/std::cos(pitch)/std::cos(pitch)*delta_pitch + std::cos(roll)*std::sin(pitch)/std::cos(pitch)/std::cos(pitch)*delta_yaw;
	G(5,5) = 1.0;
	G(0,3) = delta_position(1)*(std::cos(roll)*std::sin(pitch)*std::cos(yaw) + std::sin(roll)*std::sin(yaw)) + delta_position(2)*(-std::sin(roll)*std::sin(pitch)*std::cos(yaw) + std::cos(roll)*std::sin(yaw));
	G(0,4) = delta_position(0)*(-std::sin(pitch)*std::cos(yaw)) + delta_position(1)*(std::sin(roll)*std::cos(pitch)*std::cos(yaw)) + delta_position(2)*(std::cos(roll)*std::cos(pitch)*std::cos(yaw));
	G(0,5) = delta_position(0)*(-std::cos(pitch)*std::sin(yaw)) + delta_position(1)*(-std::sin(roll)*std::sin(pitch)*std::sin(yaw) - std::cos(roll)*std::cos(yaw)) + delta_position(2)*(-std::cos(roll)*std::sin(pitch)*std::sin(yaw) + std::sin(roll)*std::cos(yaw));
	G(1,3) = delta_position(1)*(std::cos(roll)*std::sin(pitch)*std::sin(yaw ) - std::sin(roll)*std::cos(yaw)) + delta_position(2)*(-std::sin(roll)*std::sin(pitch)*std::sin(yaw) - std::cos(roll)*std::cos(yaw));
	G(1,4) = delta_position(0)*(-std::sin(pitch)*std::sin(yaw)) + delta_position(1)*(std::sin(roll)*std::cos(pitch)*std::sin(yaw)) + delta_position(2)*(std::cos(roll)*std::cos(pitch)*std::sin(yaw));
	G(1,5) = delta_position(0)*(std::cos(pitch)*std::cos(yaw)) + delta_position(1)*(std::sin(roll)*std::sin(pitch)*std::cos(yaw) - std::cos(roll)*std::sin(yaw)) + delta_position(2)*(std::cos(roll)*std::sin(pitch)*std::cos(yaw) + std::sin(roll)*std::sin(yaw));
	G(2,3) = delta_position(1)*(std::cos(roll)*std::cos(pitch)) + delta_position(2)*(-std::sin(roll)*std::cos(pitch));
	G(2,4) = delta_position(0)*(-std::cos(pitch)) + delta_position(1)*(-std::sin(roll)*std::sin(pitch)) + delta_position(2)*(-std::cos(roll)*std::sin(pitch)) ;
	G(2,5) = 0.0;

	Eigen::MatrixXd Q(X_.size(),X_.size());
	Q.setZero();
	Q.block(0,0,3,3) = SIGMA_ODOM_*Eigen::Matrix3d::Identity();
	Q.block(3,3,3,3) = SIGMA_IMU_*Eigen::Matrix3d::Identity();

	X_ = F;
	P_ = G*P_*G.transpose() + Q;
}

void EKF::respawn()
{
	ekf_pose_.pose.position.x = respawn_pose_.pose.position.x;
	ekf_pose_.pose.position.y = respawn_pose_.pose.position.y;
	ekf_pose_.pose.position.z = ndt_pose_.pose.position.z;
	ekf_pose_.pose.orientation = ndt_pose_.pose.orientation;
	ekf_pose_.header.frame_id = map_frame_id_;
	X_(0) = respawn_pose_.pose.position.x;
	X_(1) = respawn_pose_.pose.position.y;
	// ekf_pose_pub_.publish(ekf_pose_);
	ekf_pose_pub_->publish(ekf_pose_);
}