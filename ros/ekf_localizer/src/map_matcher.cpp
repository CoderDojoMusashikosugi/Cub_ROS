#include "map_matcher/map_matcher.h"

MapMatcher::MapMatcher() : Node("MapMatcher") 
{
	this->declare_parameter<std::string>("pcd_file_path", "/home/cub/colcon_ws/src/cub/ekf_localizer/pcd/map_msakosu.pcd");
	this->declare_parameter<std::string>("pc_topic_name", {"/velodyne_points"});
	this->declare_parameter<std::string>("orb_pc_topic_name", "orb_pc_in");
	this->declare_parameter<std::string>("ekf_pose_topic_name", {"/ekf_pose"});
	this->declare_parameter<std::string>("ndt_pose_topic_name", {"/ndt_pose"});
	this->declare_parameter<std::string>("map_topic_name", {"map_out"});
	this->declare_parameter<std::string>("ndt_pc_topic_name", {"/ndt_pc"});
	this->declare_parameter<std::string>("map_frame_id", {"map"});
	this->declare_parameter<bool>("is_publish_map", {true});
	this->declare_parameter<bool>("is_pcl_offset", {false});

	this->get_parameter("pcd_file_path", pcd_file_path_);
	this->get_parameter("pc_topic_name", pc_topic_name_);
	this->get_parameter("orb_pc_topic_name", orb_pc_topic_name_);
	this->get_parameter("ekf_pose_topic_name", ekf_pose_topic_name_);
	this->get_parameter("ndt_pose_topic_name", ndt_pose_topic_name_);
	this->get_parameter("map_topic_name", map_topic_name_);
	this->get_parameter("ndt_pc_topic_name", ndt_pc_topic_name_);
	this->get_parameter("map_frame_id", map_frame_id_);
	this->get_parameter("is_publish_map", is_publish_map_);
	this->get_parameter("is_pcl_offset", is_pcl_offset_);

	this->declare_parameter<double>("VOXEL_SIZE", {0.2});
	this->declare_parameter<double>("LIMIT_RANGE", {20.0});
	this->declare_parameter<double>("TRANS_EPSILON", {0.001});
	this->declare_parameter<double>("STEP_SIZE", {0.1});
	this->declare_parameter<double>("RESOLUTION", {0.5});
	this->declare_parameter<double>("MAX_ITERATION", {35});
	this->declare_parameter<double>("MATCHING_SCORE_TH", {0.1});
	this->declare_parameter<double>("MAP_OFFSET_X", {0.0});
	this->declare_parameter<double>("MAP_OFFSET_Y", {0.0});
	this->declare_parameter<double>("MAP_OFFSET_Z", {0.0});
	this->declare_parameter<double>("MAP_OFFSET_ROLL", {0.0});
	this->declare_parameter<double>("MAP_OFFSET_PITCH", {0.0});
	this->declare_parameter<double>("MAP_OFFSET_YAW", {0.0});

	this->get_parameter("VOXEL_SIZE", VOXEL_SIZE_);
	this->get_parameter("LIMIT_RANGE", LIMIT_RANGE_);
	this->get_parameter("TRANS_EPSILON", TRANS_EPSILON_);
	this->get_parameter("STEP_SIZE", STEP_SIZE_);
	this->get_parameter("RESOLUTION", RESOLUTION_);
	this->get_parameter("MAX_ITERATION", MAX_ITERATION_);
	this->get_parameter("MATCHING_SCORE_TH", MATCHING_SCORE_TH_);
	this->get_parameter("MAP_OFFSET_X", MAP_OFFSET_X_);
	this->get_parameter("MAP_OFFSET_Y", MAP_OFFSET_Y_);
	this->get_parameter("MAP_OFFSET_Z", MAP_OFFSET_Z_);
	this->get_parameter("MAP_OFFSET_ROLL", MAP_OFFSET_ROLL_);
	this->get_parameter("MAP_OFFSET_PITCH", MAP_OFFSET_PITCH_);
	this->get_parameter("MAP_OFFSET_YAW", MAP_OFFSET_YAW_);

    pc_sub_  = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        pc_topic_name_, rclcpp::QoS(1).reliable(),
        std::bind(&MapMatcher::pc_callback, this, std::placeholders::_1));
    ekf_pose_sub_  = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        ekf_pose_topic_name_, rclcpp::QoS(1).reliable(),
        std::bind(&MapMatcher::ekf_pose_callback, this, std::placeholders::_1));

    ndt_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        ndt_pose_topic_name_, rclcpp::QoS(1).reliable());
    map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        map_topic_name_, rclcpp::QoS(1).reliable());
    ndt_pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        ndt_pc_topic_name_, rclcpp::QoS(1).reliable());


	tfBuffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);

	std::cout << "is_publish_map_: " << is_publish_map_ << std::endl;
	std::cout << "MATCHING_SCORE_TH_: " << MATCHING_SCORE_TH_ << std::endl;
	map_pcl_ = std::make_shared<PointCloudType>();
	current_pcl_ = std::make_shared<PointCloudType>();

}

MapMatcher::~MapMatcher(){}

void MapMatcher::pc_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg)
{
	pc_time_ = msg->header.stamp;
	pc_ = *msg;
	// pcl::PointCloud<pcl::PointXYZI>::Ptr raw_current_pcl(new pcl::PointCloud<pcl::PointXYZI>);
	PointCloudTypePtr raw_current_pcl(new PointCloudType);
	pcl::fromROSMsg(*msg, *raw_current_pcl);

	// downsampling
	if(VOXEL_SIZE_ > 0) downsample_pcl(raw_current_pcl,current_pcl_, VOXEL_SIZE_);
	else current_pcl_ = raw_current_pcl;

	current_pcl_->is_dense = false;
	current_pcl_->width = current_pcl_->size();

	// offset
	if(is_pcl_offset_){
		geometry_msgs::msg::TransformStamped transform_stamped;
		try{
			// lookupTransform("変換のベースとなる座標系","変更したい対象の座標系",変更したい時間(過去データを扱う場合は注意が必要))
			transform_stamped = tfBuffer_->lookupTransform("base_link", "map", tf2::TimePointZero); //座標系の変換 

			// transform_stamped = buffer_->lookupTransform("base_link", msg->header.frame_id, rclcpp::Time(0)); 
		}
		catch(tf2::TransformException& ex){
			// ROS_WARN("%s", ex.what());
			return;
		}	
		Eigen::Matrix4f transform = tf2::transformToEigen(transform_stamped.transform).matrix().cast<float>();
		pcl::transformPointCloud(*current_pcl_, *current_pcl_, transform);
	}
	has_received_pc_ = true;
}

/*
void MapMatcher::orb_pc_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
	orb_pc_ =*msg;
}
*/

void MapMatcher::ekf_pose_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg)
{
	ekf_pose_ = *msg;
	has_received_ekf_pose_ = true;
}

/*
void MapMatcher::map_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{

}
*/

void MapMatcher::init_map() 
{
	map_pcl_->clear(); 
}

void MapMatcher::set_pcl(pcl::PointCloud<pcl::PointXYZI>::Ptr input_pcl,pcl::PointCloud<pcl::PointXYZI>::Ptr& output_pcl,double x,double y,double z)
{
	output_pcl->clear();
	std::cout << "input_pcl size: " << input_pcl->points.size() << std::endl;
	
	pcl::PassThrough<pcl::PointXYZI> pass;
	pass.setInputCloud(input_pcl);
	pass.setFilterFieldName("x");
	pass.setFilterLimits(-LIMIT_RANGE_ + x, LIMIT_RANGE_ + x);
	pass.filter(*output_pcl);
	std::cout << "output_pcl size x: " << output_pcl->points.size() << std::endl;

	pass.setInputCloud(output_pcl);
	pass.setFilterFieldName("y");
	pass.setFilterLimits(-LIMIT_RANGE_ + y, LIMIT_RANGE_ + y);
	pass.filter(*output_pcl);
	std::cout << "output_pcl size y: " << output_pcl->points.size() << std::endl;

	pass.setInputCloud(output_pcl);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(-15 + z, 15 + z);
	pass.filter(*output_pcl);
	std::cout << "output_pcl size z: " << output_pcl->points.size() << std::endl;

}

void MapMatcher::read_map()
{
	// initialize
	if(is_reset_){
		init_map();
		is_reset_ = false;
	}

	// load map
	// pcl::PointCloud<pcl::PointXYZI>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZI>);
	PointCloudTypePtr raw_cloud(new PointCloudType);
	if(pcd_file_path_ == ""){
		// ROS_ERROR("No map entered");
		std::cout << "No MAP" << std::endl;
		return;
	}
	if(pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_file_path_,*raw_cloud) == -1){
		// ROS_ERROR("Cloud not find a map");
		std::cout << "could not read map" << std::endl;
		return;
	}
	else std::cout << "Load: " << pcd_file_path_ << std::endl;

	std::cout  << "raw map_points: " << raw_cloud->points.size() << std::endl;
	// downsampling
	// if(VOXEL_SIZE_ > 0) downsample_pcl(raw_cloud,map_pcl_,VOXEL_SIZE_);
	// else map_pcl_ = raw_cloud;
	map_pcl_ = raw_cloud;
	std::cout  << "down map_points: " << map_pcl_->points.size() << std::endl;

	// offset
	Eigen::Vector3f offset_position(MAP_OFFSET_X_,MAP_OFFSET_Y_,MAP_OFFSET_Z_);
	Eigen::Quaternionf offset_orientation = msg_to_quat_eigen(rpy_to_msg(MAP_OFFSET_ROLL_,MAP_OFFSET_PITCH_,MAP_OFFSET_YAW_));
	pcl::transformPointCloud(*map_pcl_,*map_pcl_,offset_position,offset_orientation);

	// publish map
	if(is_publish_map_){
		sensor_msgs::msg::PointCloud2 map;
		pcl::toROSMsg(*map_pcl_, map);
		//map.header.stamp = ros::Time(0);
		// map.header.frame_id = map_frame_id_;
		int count_pub = 0;
		map.header.frame_id = "map";
		map_pub_->publish(map);
		// while(count_pub < 5000){
			map_pub_->publish(map);
			count_pub ++;
		// }
		std::cout << "publish map point cloud!!" << std::endl;
	}

	has_read_map_ = true;
}

void MapMatcher::downsample_pcl(pcl::PointCloud<pcl::PointXYZI>::Ptr input_pcl,pcl::PointCloud<pcl::PointXYZI>::Ptr& output_pcl, double voxel_size)
{
	pcl::VoxelGrid<pcl::PointXYZI> voxel_sampler;
	voxel_sampler.setLeafSize(voxel_size,voxel_size,voxel_size);
	voxel_sampler.setInputCloud(input_pcl);
	voxel_sampler.filter(*output_pcl);
}

void MapMatcher::matching(pcl::PointCloud<pcl::PointXYZI>::Ptr map_pcl,pcl::PointCloud<pcl::PointXYZI>::Ptr local_pcl)
{
	// passthrough
	// pcl::PointCloud<pcl::PointXYZI>::Ptr map_local_pcl(new pcl::PointCloud<pcl::PointXYZI>);
	// pcl::PointCloud<pcl::PointXYZI>::Ptr current_local_pcl(new pcl::PointCloud<pcl::PointXYZI>);
	PointCloudTypePtr map_local_pcl(new PointCloudType);
	PointCloudTypePtr current_local_pcl(new PointCloudType);
	std::cout << "map_pcl_ size: " << map_pcl_->points.size() << std::endl;
	set_pcl(map_pcl_, map_local_pcl, ekf_pose_.pose.position.x, ekf_pose_.pose.position.y, ekf_pose_.pose.position.z);
	std::cout << "map_local_pcl_ size: " << map_local_pcl->points.size() << std::endl;
	std::cout << "cur_pcl_ size: " << current_pcl_->points.size() << std::endl;
	set_pcl(current_pcl_,current_local_pcl,0.0,0.0,0.0);
	std::cout << "cur_local_pcl_ size: " << current_local_pcl->points.size() << std::endl;

	// initialize
	//Eigen::AngleAxisf init_rotation(msg_to_quat_eigen(ekf_pose_.pose.orientation));
	Eigen::AngleAxisf init_rotation((float)get_yaw_from_quat(ekf_pose_.pose.orientation),Eigen::Vector3f::UnitZ());
	Eigen::Translation3f init_translation((float)ekf_pose_.pose.position.x,(float)ekf_pose_.pose.position.y,(float)ekf_pose_.pose.position.z);
	Eigen::Matrix4f init_guess = (init_translation*init_rotation).matrix();

	// align
	// pcl::PointCloud<pcl::PointXYZI>::Ptr ndt_pcl(new pcl::PointCloud<pcl::PointXYZI>);
	PointCloudTypePtr ndt_pcl(new PointCloudType);
	pcl::NormalDistributionsTransform<pcl::PointXYZI,pcl::PointXYZI> ndt;
	ndt.setTransformationEpsilon(TRANS_EPSILON_);
	ndt.setStepSize(STEP_SIZE_);
	ndt.setResolution(RESOLUTION_);
	ndt.setMaximumIterations(MAX_ITERATION_);
	if(map_local_pcl->points.empty() || current_local_pcl->points.empty()){
		if(map_local_pcl->points.empty()) std::cout << "map_local_pcl is empty" << std::endl;
		if(current_local_pcl->points.empty()) std::cout << "current_local_pcl is empty" << std::endl;
		return;
	}
	ndt.setInputTarget(map_local_pcl);
	ndt.setInputSource(current_local_pcl);
	if(current_local_pcl->points.size() > map_local_pcl->points.size()){
		std::cout << "local clouds > map clouds" << std::endl;
		return;
	}
	ndt.align(*ndt_pcl,init_guess);
	//ndt.align(*ndt_pcl,Eigen::Matrix4f::Identity());
	if(!ndt.hasConverged()){
		std::cout << "Has converged" << std::endl;
		return;
	}

	std::cout << "FitnessScore: " << ndt.getFitnessScore() << std::endl;
	std::cout << std::endl;

	if(ndt.getFitnessScore() <= MATCHING_SCORE_TH_){
		Eigen::Matrix4f translation = ndt.getFinalTransformation();	
		Eigen::Quaternionf quaternion(Eigen::Matrix3f(translation.block(0,0,3,3)));
		quaternion.normalize();

		// publish_ndt_pose
		geometry_msgs::msg::PoseStamped ndt_pose;
		ndt_pose.pose.position.x = translation(0,3);
		ndt_pose.pose.position.y = translation(1,3);
		ndt_pose.pose.position.z = translation(2,3);
		//ndt_pose.pose.position.z = 0.0;
		ndt_pose.pose.orientation = quat_eigen_to_msg(quaternion);
		ndt_pose.header.stamp = ekf_pose_.header.stamp;
		ndt_pose.header.frame_id = ekf_pose_.header.frame_id;
		ndt_pose_pub_->publish(ndt_pose);

		// publish ndt_pcl
		sensor_msgs::msg::PointCloud2 ndt_msg;
		pcl::toROSMsg(*ndt_pcl,ndt_msg);
		ndt_msg.header.stamp = pc_time_;
		ndt_msg.header.frame_id = map_frame_id_;
		ndt_pc_pub_->publish(ndt_msg);

		//debug
		double roll, pitch, yaw;
		tf2::Quaternion q;
		tf2::fromMsg(ndt_pose.pose.orientation,q);
		tf2::Matrix3x3 r(q);
		r.getRPY(roll,pitch,yaw);
		std::cout << "NDT POSE: " << std::endl;
		std::cout << " X : " << ndt_pose.pose.position.x << std::endl;
		std::cout << " Y : " << ndt_pose.pose.position.y << std::endl;
		std::cout << "YAW: " << yaw << std::endl;
		std::cout << std::endl;
	}
	else{
		std::cout << "Cannot match due to high sum of squared distance between clouds" << std::endl;
	}
}

double MapMatcher::get_yaw_from_quat(geometry_msgs::msg::Quaternion q)
{
	double r, p, y;
	tf2::Quaternion quaternion(q.x,q.y,q.z,q.w);
	tf2::Matrix3x3(quaternion).getRPY(r,p,y);

	return y;
}

geometry_msgs::msg::Quaternion MapMatcher::rpy_to_msg(double roll, double pitch, double yaw)
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

geometry_msgs::msg::Quaternion MapMatcher::quat_eigen_to_msg(Eigen::Quaternionf q)
{
	geometry_msgs::msg::Quaternion msg;
	msg.x = (double)q.x();
	msg.y = (double)q.y();
	msg.z = (double)q.z();
	msg.w = (double)q.w();

	return msg;
}

Eigen::Quaternionf MapMatcher::msg_to_quat_eigen(geometry_msgs::msg::Quaternion q)
{
	Eigen::Quaternionf quaternion;
	quaternion.x() = (float)q.x;
	quaternion.y() = (float)q.y;
	quaternion.z() = (float)q.z;
	quaternion.w() = (float)q.w;
	quaternion.normalize();

	return quaternion;
}

void MapMatcher::process()
{
	if(has_read_map_ && has_received_ekf_pose_ && has_received_pc_){
		std::cout << "is_read_map: " << has_read_map_ << std::endl;
		std::cout << "is_ekf_pose: " << has_received_ekf_pose_ << std::endl;
		std::cout << "is_rec_pc: " << has_received_pc_ << std::endl;
		matching(map_pcl_,current_pcl_);
		has_received_pc_ = false;
		has_received_ekf_pose_ = false;
	}
	else if(has_read_map_) std::cout << "Waiting msg" << std::endl;
}

int main(int argc,char** argv)
{
	// ros::init(argc,argv,"map_matcher");
	// MapMatcher matcher;
	std::cout << "---map_matcher---" << std::endl;
    rclcpp::init(argc, argv); // ノードの初期化
    auto node = std::make_shared<MapMatcher>();
	// std::cout << "before read_map" << std::endl;
	node->read_map();
	// std::cout << "after read_map" << std::endl;
	rclcpp::Rate rate(10.0);
	while(rclcpp::ok()){
		node->process();
		rclcpp::spin_some(node);
		rate.sleep();
	}
	return 0;
}