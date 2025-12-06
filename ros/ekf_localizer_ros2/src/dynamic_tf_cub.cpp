#include "tf/dynamic_tf_cub.h"
DynamicTfCub::DynamicTfCub() : Node("DynamicTFCub")
{
    declare_parameter<std::string>("base_frame_id", "base_link");
    declare_parameter<std::string>("odom_frame_id", "odom");
    declare_parameter<std::string>("map_frame_id", "map");
    declare_parameter<bool>("odom_tf_enable", false);  // true: map->odom->base_link, false: map->base_link

    odom_tf_enable_ = this->get_parameter("odom_tf_enable").as_bool();

    /*subscriber*/
    // sub_current_pose = nh.subscribe("/ekf_pose", 10, &DynamicTfCub::current_pose_callback, this);
    sub_current_pose   = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/test/ekf_pose", rclcpp::QoS(1).reliable(),
        std::bind(&DynamicTfCub::current_pose_callback, this, std::placeholders::_1));

    // odom_tf_enable=trueの場合のみオドメトリをサブスクライブ
    if (odom_tf_enable_) {
        sub_wheel_odometry = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", rclcpp::QoS(1).reliable(),
            std::bind(&DynamicTfCub::wheel_odometry_callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "TF mode: map -> odom -> base_link");
    } else {
        RCLCPP_INFO(this->get_logger(), "TF mode: map -> base_link (direct)");
    }

	tfBuffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    // dynamic_br_ = std::make_shared<tf2_ros::TransformBroadcaster>(*tfBuffer_);
    dynamic_br_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    RCLCPP_INFO(this->get_logger(), "done setting TF");
}
DynamicTfCub::~DynamicTfCub(){}
void DynamicTfCub::current_pose_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg)
{
    // RCLCPP_INFO(this->get_logger(), "Received EKF Pose");
    current_pose = *msg;
    
    // pub_static_tf();
}
void DynamicTfCub::wheel_odometry_callback(const nav_msgs::msg::Odometry::ConstSharedPtr& msg)
{
    // RCLCPP_DEBUG(this->get_logger(), "Received Wheel Odometry");
    wheel_odometry_pose = *msg;
    if (odom_tf_enable_) {
        pub_wheel_odometry_tf();  // odom -> base_link
    }
}

void DynamicTfCub::pub_wheel_odometry_tf()
{
    geometry_msgs::msg::TransformStamped transformStamped;
    rclcpp::Clock clock(RCL_ROS_TIME);  // ROS時間を使用
    // transformStamped.header.stamp = clock.now();
    transformStamped.header.stamp = wheel_odometry_pose.header.stamp;
    transformStamped.header.frame_id = "odom";
    transformStamped.child_frame_id = "base_link";
    transformStamped.transform.translation.x = wheel_odometry_pose.pose.pose.position.x;
    transformStamped.transform.translation.y = wheel_odometry_pose.pose.pose.position.y;
    transformStamped.transform.translation.z = 0.0;

    // tf2::Quaternion q;
    // q.setRPY(0, 0, 0.0); // YAW
    transformStamped.transform.rotation.x = wheel_odometry_pose.pose.pose.orientation.x;
    transformStamped.transform.rotation.y = wheel_odometry_pose.pose.pose.orientation.y;
    transformStamped.transform.rotation.z = wheel_odometry_pose.pose.pose.orientation.z;
    transformStamped.transform.rotation.w = wheel_odometry_pose.pose.pose.orientation.w;
    dynamic_br_->sendTransform(transformStamped);
    RCLCPP_DEBUG(this->get_logger(), "send odom->base_link TF");
}

// map -> base_link (odom_tf_enable=false時に使用)
void DynamicTfCub::pub_direct_tf()
{
    geometry_msgs::msg::TransformStamped transformStamped;
    transformStamped.header.stamp = current_pose.header.stamp;
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
    RCLCPP_DEBUG(this->get_logger(), "send map->base_link TF (direct)");
}

// map -> odom (odom_tf_enable=true時に使用)
void DynamicTfCub::pub_dynamic_tf()
{
    geometry_msgs::msg::TransformStamped transformStamped;
    rclcpp::Clock clock(RCL_ROS_TIME);  // ROS時間を使用
    // transformStamped.header.stamp = clock.now();
    // transformStamped.header.stamp =this->get_clock()->now();
    transformStamped.header.stamp = current_pose.header.stamp;
    transformStamped.header.frame_id = "map";
    transformStamped.child_frame_id = "odom";
    transformStamped.transform.translation.x = current_pose.pose.position.x - wheel_odometry_pose.pose.pose.position.x;
    transformStamped.transform.translation.y = current_pose.pose.position.y - wheel_odometry_pose.pose.pose.position.y;
    transformStamped.transform.translation.z = 0.0;

    // tf2::Quaternion q;
    // q.setRPY(0, 0, 0.0); // YAW
    transformStamped.transform.rotation.x = current_pose.pose.orientation.x - wheel_odometry_pose.pose.pose.orientation.x;
    transformStamped.transform.rotation.y = current_pose.pose.orientation.y - wheel_odometry_pose.pose.pose.orientation.y;
    transformStamped.transform.rotation.z = current_pose.pose.orientation.z - wheel_odometry_pose.pose.pose.orientation.z;
    transformStamped.transform.rotation.w = current_pose.pose.orientation.w - wheel_odometry_pose.pose.pose.orientation.w;
    dynamic_br_->sendTransform(transformStamped);
    RCLCPP_DEBUG(this->get_logger(), "send map->odom TF");
}

void DynamicTfCub::process()
{
    if (odom_tf_enable_) {
        pub_dynamic_tf();  // map -> odom
    } else {
        pub_direct_tf();   // map -> base_link
    }
}

int main (int argc,char **argv)
{
    // ros::init(argc, argv, "dynamic_tf_cub");
    rclcpp::init(argc, argv); // ノードの初期化
    auto node = std::make_shared<DynamicTfCub>();
    // DynamicTfCub dynamic_tf_cub;
	while(rclcpp::ok()){
        node->process();
		rclcpp::spin_some(node);
	}
    // rclcpp::spin(node);
    return 0;
}
