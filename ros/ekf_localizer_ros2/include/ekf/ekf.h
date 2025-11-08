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
#include "nav_msgs/msg/path.hpp"

#include <tf2/utils.h>
#include <tf2_eigen/tf2_eigen.hpp>

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
	void gps_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg);
	void initialpose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg);
	
	// void measurement_callback(const std_msgs::msg::Bool::ConstSharedPtr msg);

	void initialize(double x,double y,double z,double roll,double pitch,double yaw);
	void set_pose(double x,double y,double z,double roll,double pitch,double yaw);
	void calc_rpy_from_quat(geometry_msgs::msg::Quaternion q,double& roll,double& pitch,double& yaw);
	
	void motion_update_3DoF(double dt);
	void motion_update_6DoF(double dt);
	void motion_update(double dt);
	void motion_update_by_imu(double dt);
	void motion_update_by_odom(double dt);
	void measurement_update_3DoF();
	
	void publish_ekf_pose();
	// void publish_tf();

	double normalize_angle(double angle);
	double calc_yaw_from_quat(geometry_msgs::msg::Quaternion q);
	geometry_msgs::msg::Quaternion rpy_to_msg(double roll,double pitch,double yaw);
	Eigen::Matrix3d calc_rotation_matrix(Eigen::Vector3d euler_angle);
	Eigen::VectorXd measurement_function(Eigen::VectorXd x,Eigen::MatrixXd h);
	void measurement_update_gps();
	bool check_mahalanobis_distance(geometry_msgs::msg::PoseStamped ekf_pose, geometry_msgs::msg::PoseStamped ndt_pose);
	bool check_ekf_covariance(geometry_msgs::msg::PoseStamped ekf_pose);
    void reset_ekf(double x, double y, double yaw);

	// double get_yaw(geometry_msgs::msg::Quaternion q);

	rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr gps_pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ndt_pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
	rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_sub_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ekf_pose_pub_;
	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;


	std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
    std::shared_ptr<tf2_ros::TransformListener> listener_;
	std::shared_ptr<tf2::BufferCore> buffer_ = std::make_shared<tf2::BufferCore>();

	nav_msgs::msg::Odometry odom_;
	nav_msgs::msg::Odometry last_odom_pose;
	Eigen::Vector3d last_odom_eigen = Eigen::Vector3d::Zero();
	geometry_msgs::msg::PoseStamped ndt_pose_;
	geometry_msgs::msg::PoseStamped ekf_pose_;
	sensor_msgs::msg::Imu imu_;
	rclcpp::Time now_time_odom_;
	rclcpp::Time last_time_odom_;
	rclcpp::Time now_time_imu_;
	rclcpp::Time last_time_imu_;
	rclcpp::Time time_publish_;

	std_msgs::msg::Bool is_measurement_;
	nav_msgs::msg::Path ekf_pose_trajectry;
	std::vector<geometry_msgs::msg::PoseStamped> poses_;
	geometry_msgs::msg::PoseWithCovarianceStamped gps_pose_;


	std::string ndt_pose_topic_name_;
	std::string imu_topic_name_;
	std::string odom_topic_name_;
	std::string ekf_pose_topic_name_;
	std::string map_frame_id_;
	std::string odom_frame_id_;
	std::string base_link_frame_id_;
	std::string gps_pose_topic_name_;
	std::string initialpose_topic_name_;

	std::string measurement_topic_name_;

	bool suppress_measurements_after_reset_ = false;
    rclcpp::Time measurement_suppress_until_;
    double measurement_suppress_duration_ = 2.0;

	bool has_received_odom_;
	bool has_received_imu_;
	bool has_received_ndt_pose_;
	bool has_received_gps_;
	// bool is_first_;
	bool is_odom_tf_;

	bool is_first_odom_ = true;
	bool is_first_imu_ = true;
	bool gps_measurement_enable_ = false;
	bool ndt_measurement_enable_ = false;

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
	// double dt;
	double th_mahalanobis_;
	double th_covariance_;
	double th_pose_covariance_;
	double th_direction_covariance_;
	double dt_;
	double SIGMA_GPS_;
	double ekf_hz_;

	int STATE_SIZE_;
	Eigen::VectorXd X_;
	Eigen::MatrixXd P_;

	// rclcpp::Time last_time_;
    bool first_callback_ = true;
	geometry_msgs::msg::Point last_position_;
	Eigen::Vector3d last_odom_pose_ = Eigen::Vector3d::Zero();
	double last_odom_yaw_;
};
#endif	// EKF_H_