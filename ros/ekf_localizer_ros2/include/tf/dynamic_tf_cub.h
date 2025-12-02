#ifndef __DYNAMIC_TF_CUB_H
#define __DYNAMIC_TF_CUB_H
// #include <ros/ros.h>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/buffer.h>

class DynamicTfCub : public rclcpp::Node
{
public:
    DynamicTfCub();
    ~DynamicTfCub();
    void process();

private:
    //method
    void pub_dynamic_tf();        // map->odom TF (when odom_tf_enable=true)
    void pub_direct_tf();         // map->base_link TF (when odom_tf_enable=false)
    void pub_static_tf();
    void pub_wheel_odometry_tf(); // odom->base_link TF (when odom_tf_enable=true)
    void wheel_odometry_callback(const nav_msgs::msg::Odometry::ConstSharedPtr&);
    void current_pose_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr&);

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_current_pose;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_wheel_odometry;

    geometry_msgs::msg::PoseStamped current_pose;
    nav_msgs::msg::Odometry wheel_odometry_pose;

    bool odom_tf_enable_;  // true: map->odom->base_link, false: map->base_link

    std::shared_ptr<tf2_ros::TransformBroadcaster> dynamic_br_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> static_br_;
	std::unique_ptr<tf2_ros::Buffer> tfBuffer_;

};

#endif
