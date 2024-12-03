#ifndef MAP_MATCHER_H_
#define MAP_MATCHER_H_

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/correspondence_types.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include "tf2_ros/transform_listener.h"

#include <Eigen/Dense>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class MapMatcher : public rclcpp::Node
{
public:
	MapMatcher();
	~MapMatcher();
	void process();
	void read_map();
    typedef pcl::PointXYZI PointType;
    typedef pcl::PointCloud<PointType> PointCloudType;
    typedef pcl::PointCloud<PointType>::Ptr PointCloudTypePtr;
private:

	void pc_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg);
	void orb_pc_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg);
	void ekf_pose_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg);
	void map_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg);


 	void init_map();

	void set_pcl(pcl::PointCloud<pcl::PointXYZI>::Ptr input_pcl, pcl::PointCloud<pcl::PointXYZI>::Ptr& output_pcl, double x, double y, double z);
	void downsample_pcl(pcl::PointCloud<pcl::PointXYZI>::Ptr input_pcl, pcl::PointCloud<pcl::PointXYZI>::Ptr& output_pcl, double voxel_size);
	void matching(pcl::PointCloud<pcl::PointXYZI>::Ptr map_pcl, pcl::PointCloud<pcl::PointXYZI>::Ptr local_pcl);

	double get_yaw_from_quat(geometry_msgs::msg::Quaternion q);
	geometry_msgs::msg::Quaternion rpy_to_msg(double roll, double pitch, double yaw);
	geometry_msgs::msg::Quaternion quat_eigen_to_msg(Eigen::Quaternionf q);
	Eigen::Quaternionf msg_to_quat_eigen(geometry_msgs::msg::Quaternion q);

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr orb_pc_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ekf_pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ndt_pose_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ndt_pc_pub_;

	std::unique_ptr<tf2_ros::Buffer> tfBuffer_;
    std::shared_ptr<tf2_ros::TransformListener> tfListener_;
	PointCloudTypePtr map_pcl_;
	PointCloudTypePtr current_pcl_;

	sensor_msgs::msg::PointCloud2 pc_;
	sensor_msgs::msg::PointCloud2 orb_pc_;
	geometry_msgs::msg::PoseStamped ekf_pose_;
	rclcpp::Time pc_time_;
	bool is_reset_ = true;
	bool is_start_;
	bool has_received_ekf_pose_ = false;
	bool has_received_pc_ = false;
	bool has_read_map_ = false;
	bool is_first;
	bool is_publish_map_;

	// parameter
	std::string pcd_file_path_;
	std::string pc_topic_name_;
	std::string orb_pc_topic_name_;
	std::string ekf_pose_topic_name_;
	std::string ndt_pose_topic_name_;
	std::string map_topic_name_;
	std::string ndt_pc_topic_name_;
	std::string map_frame_id_;

	bool is_pcl_offset_;

	double VOXEL_SIZE_;
	double LIMIT_RANGE_;
	double TRANS_EPSILON_;
	double STEP_SIZE_;
	double RESOLUTION_;
	double MAX_ITERATION_;
	double MATCHING_SCORE_TH_;
	double MAP_OFFSET_X_;
	double MAP_OFFSET_Y_;
	double MAP_OFFSET_Z_;
	double MAP_OFFSET_ROLL_;
	double MAP_OFFSET_PITCH_;
	double MAP_OFFSET_YAW_;
};

#endif	// MAP_MATCHER_H_