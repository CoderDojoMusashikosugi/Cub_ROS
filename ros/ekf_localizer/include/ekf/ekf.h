#ifndef EKF_H_
#define EKF_H_
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <std_msgs/msg/bool.hpp>

#include <Eigen/Dense>

class EKF : public rclcpp::Node
{
public:
	EKF();
	~EKF();
	void process();

private:
	void ndt_pose_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);
	void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
	void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr msg);
	
	void measurement_callback(const std_msgs::msg::Bool::ConstSharedPtr msg);

	void initialize(double x,double y,double z,double roll,double pitch,double yaw);
	void set_pose(double x,double y,double z,double roll,double pitch,double yaw);
	void calc_rpy_from_quat(geometry_msgs::msg::Quaternion q,double& roll,double& pitch,double& yaw);
	
	void motion_update_3DoF(double dt);
	void motion_update_6DoF(double dt);
	void motion_update(double dt);
	void measurement_update();
	void measurement_update_3DoF();
	void measurement_update_6DoF();
	
	void respawn();

	void publish_ekf_pose();
	// void publish_tf();

	double normalize_angle(double angle);
	double calc_yaw_from_quat(geometry_msgs::msg::Quaternion q);
	geometry_msgs::msg::Quaternion rpy_to_msg(double roll,double pitch,double yaw);
	Eigen::Matrix3d calc_rotation_matrix(Eigen::Vector3d euler_angle);
	Eigen::VectorXd measurement_function(Eigen::VectorXd x,Eigen::MatrixXd h);

	// double get_yaw(geometry_msgs::Quaternion q);

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ndt_pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ekf_pose_pub_;


	std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
    std::shared_ptr<tf2_ros::TransformListener> listener_;
	std::shared_ptr<tf2::BufferCore> buffer_ = std::make_shared<tf2::BufferCore>();


	nav_msgs::msg::Odometry odom_;
	Eigen::Vector3d last_odom_eigen = Eigen::Vector3d::Zero();
	nav_msgs::msg::Odometry last_odom_pose;
	geometry_msgs::msg::PoseStamped ndt_pose_;
	geometry_msgs::msg::PoseStamped ekf_pose_;
	sensor_msgs::msg::Imu imu_;
	rclcpp::Time now_time_;
	rclcpp::Time last_time_;

	std_msgs::msg::Bool is_measurement_;
	geometry_msgs::msg::PoseStamped respawn_pose_;

	std::string ndt_pose_topic_name_;
	std::string imu_topic_name_;
	std::string odom_topic_name_;
	std::string ekf_pose_topic_name_;
	std::string map_frame_id_;
	std::string odom_frame_id_;
	std::string base_link_frame_id_;

	std::string measurement_topic_name_;
	std::string respawn_pose_topic_name_;

	bool has_received_odom_;
	bool has_received_imu_;
	bool has_received_ndt_pose_;
	// bool is_first_;
	bool is_odom_tf_;
	bool is_3DoF_;

	bool is_respawn_;
	bool is_first_ = true;

	double INIT_X_;
	double INIT_Y_;
	double INIT_Z_;
	double INIT_ROLL_;
	double INIT_PITCH_;
	double INIT_YAW_;
	double INIT_SIGMA_;
	double SIGMA_IMU_;
	double SIGMA_ODOM_;
	double SIGMA_NDT_;
	double MOTION_NOISE_NN_;
	double MOTION_NOISE_NO_;
	double MOTION_NOISE_ON_;
	double MOTION_NOISE_OO_;
	double dt;

	int STATE_SIZE_;
	Eigen::VectorXd X_;
	Eigen::MatrixXd P_;

	// rclcpp::Time last_time_;
    bool first_callback_ = true;
	geometry_msgs::msg::Point last_position_;
	Eigen::Vector3d last_odom_pose_ = Eigen::Vector3d::Zero();
};
#endif	// EKF_H_