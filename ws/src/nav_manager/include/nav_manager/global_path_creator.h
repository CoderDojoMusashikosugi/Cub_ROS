#ifndef __GLOBAL_PATH_CREATOR_H
#define __GLOBAL_PATH_CREATOR_H

// 受け取った順番に線を引く
#include "ros/ros.h"
#include "iostream"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Pose2D.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PointStamped.h"
#include <std_msgs/Int32MultiArray.h>
#include "nav_manager/waypoints.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
class GlobalPathCreator
{
public:
    GlobalPathCreator();
    void process();

private:
    // void path_callback(const std_msgs::Int32MultiArray::ConstPtr&);
    void load_waypoints();
    void load_route();
    void make_global_path();

    bool replan_flag;
    bool node_edge_flag;
    bool global_path_flag;
    std::vector<int> routes_;
    std::vector<Waypoint> waypoints_;

    XmlRpc::XmlRpcValue waypoints_list_;
    XmlRpc::XmlRpcValue route_list_;

    ros::NodeHandle n;
    ros::NodeHandle private_nh_;
  
    ros::Publisher pub_path;
    ros::Publisher pub_id;
    ros::Publisher pub_waypoint;

    nav_msgs::Path global_path;
    nav_msgs::Path waypoint_path;
};
#endif
