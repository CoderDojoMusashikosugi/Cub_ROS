#include "tf/dynamic_tf_cub.h"
DynamicTfCub::DynamicTfCub() : Node("DynamicTFCub")
{
    /*subscriber*/
    // sub_current_pose = nh.subscribe("/ekf_pose", 10, &DynamicTfCub::current_pose_callback, this);
    sub_current_pose   = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/ekf_pose", rclcpp::QoS(1).reliable(),
        std::bind(&DynamicTfCub::current_pose_callback, this, std::placeholders::_1));

	tfBuffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    // dynamic_br_ = std::make_shared<tf2_ros::TransformBroadcaster>(*tfBuffer_);
    dynamic_br_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
}
DynamicTfCub::~DynamicTfCub(){}
void DynamicTfCub::current_pose_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg)
{
    current_pose = *msg;
    pub_dynamic_tf();
    // pub_static_tf();
}

// void DynamicTfCub::pub_static_tf()
// {
// 	geometry_msgs::msg::TransformStamped static_transform_stamped;
// 	static_transform_stamped.header.stamp = node->get_clock()->now();
// 	static_transform_stamped.header.frame_id = "base_link";
// 	static_transform_stamped.child_frame_id = "Pandar";

// 	static_transform_stamped.transform.translation.x = 0.0;
// 	static_transform_stamped.transform.translation.y = 0.0;
// 	static_transform_stamped.transform.translation.z = 0.0;

// 	// tf2::Quaternion quaternion;
// 	static_transform_stamped.transform.rotation.x = 0.0;
// 	static_transform_stamped.transform.rotation.y = 0.0;
// 	static_transform_stamped.transform.rotation.z = 0.0;
// 	static_transform_stamped.transform.rotation.w = 1.0;

//     static_br_.sendTransform(static_transform_stamped);
// }
void DynamicTfCub::pub_dynamic_tf()
{
    geometry_msgs::msg::TransformStamped transformStamped;
    rclcpp::Clock clock(RCL_ROS_TIME);  // ROS時間を使用
    // transformStamped.header.stamp = clock.now();
    transformStamped.header.stamp =this->get_clock()->now();
    transformStamped.header.frame_id = "map";
    transformStamped.child_frame_id = "base_link";
    transformStamped.transform.translation.x = current_pose.pose.position.x;
    transformStamped.transform.translation.y = current_pose.pose.position.y;
    transformStamped.transform.translation.z = 0.0;

    // tf2::Quaternion q;
    // q.setRPY(0, 0, 0.0); // YAW
    transformStamped.transform.rotation.x = current_pose.pose.orientation.x;
    transformStamped.transform.rotation.y = current_pose.pose.orientation.y;
    transformStamped.transform.rotation.z = current_pose.pose.orientation.z;
    transformStamped.transform.rotation.w = current_pose.pose.orientation.w;
    dynamic_br_->sendTransform(transformStamped);
}

void DynamicTfCub::process()
{
    pub_dynamic_tf();
}

int main (int argc,char **argv)
{
    // ros::init(argc, argv, "dynamic_tf_cub");
    rclcpp::init(argc, argv); // ノードの初期化
    auto node = std::make_shared<DynamicTfCub>();
    // DynamicTfCub dynamic_tf_cub;
	while(rclcpp::ok()){
        // node->process();
		rclcpp::spin_some(node);
	}
    // rclcpp::spin(node);
    return 0;
}