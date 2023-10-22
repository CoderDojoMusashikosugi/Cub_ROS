#include "nav_manager/next_waypoint_creator.h"

NextWaypointCreator::NextWaypointCreator():private_nh("~")
{
    //parameter
    private_nh.param("hz",hz,{10});
    private_nh.param("border_distance",border_distance,{2.0});
    //subscriber
    sub_global_path = nh.subscribe("/global_path/path",10,&NextWaypointCreator::global_path_callback,this);
    sub_current_pose = nh.subscribe("/ekf_pose",10,&NextWaypointCreator::current_pose_callback,this);
    // sub_path = nh.subscribe("/global_path/path",10,&NextWaypointCreator::path_callback,this);

    //publisher
    pub_next_waypoint = nh.advertise<geometry_msgs::PoseStamped>("/next_waypoint",1);
    // pub_estimated_edge = nh.advertise<amsl_navigation_msgs::Edge>("/estimated_pose/edge", 1);
}

// void NextWaypointCreator::path_callback(const std_msgs::Int32MultiArray::ConstPtr& msg_path)
// {
//     std::cout<<"path callback "<<std::endl;
//     global_path_num = *msg_path;
//     have_recieved_multi_array = true;
// }
void NextWaypointCreator::global_path_callback(const nav_msgs::Path::ConstPtr& msg)
{
    std::cout<<"global_path callback "<<std::endl;
    global_path =* msg;
    std::cout<<"global_path size: " << global_path.poses.size() << std::endl;
    goal_number = 0; // 最初は0番目
    next_waypoint = global_path.poses[goal_number]; // goal_number番目の位置
    have_recieved_path = true;
}
void NextWaypointCreator::current_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    std::cout<<"current_pose callback "<<std::endl;
    current_pose = *msg;
    // if(!have_recieved_pose) 
    have_recieved_pose = true;

    // geometry_msgs::TransformStamped transformStamped;
    // transformStamped.header.stamp = ros::Time::now();
    // transformStamped.header.frame_id = "map";
    // transformStamped.child_frame_id = "base_link";
    // transformStamped.transform.translation.x = current_pose.pose.position.x;
    // transformStamped.transform.translation.y = current_pose.pose.position.y;
    // transformStamped.transform.translation.z = 0.0;

    // // tf2::Quaternion q;
    // // q.setRPY(0, 0, 0.0); // YAW
    // transformStamped.transform.rotation.x = current_pose.pose.orientation.x;
    // transformStamped.transform.rotation.y = current_pose.pose.orientation.y;
    // transformStamped.transform.rotation.z = current_pose.pose.orientation.z;
    // transformStamped.transform.rotation.w = current_pose.pose.orientation.w;
    // dynamic_br_.sendTransform(transformStamped);
}
void NextWaypointCreator::select_next_goal()
{
    double measure_distance = sqrt(pow(next_waypoint.pose.position.x-current_pose.pose.position.x,2)+pow(next_waypoint.pose.position.y-current_pose.pose.position.y,2)); // 自分の位置と次の通過ポイントの距離
    std::cout<<"distance: "<< measure_distance<<std::endl;
    if(measure_distance < border_distance) goal_number += 1;

    if(global_path.poses.size() > goal_number) next_waypoint = global_path.poses[goal_number];
    else next_waypoint = global_path.poses[global_path.poses.size()-1];

    if(goal_number == 0) goal_number ++; // スタート地点ならすぐ更新
    std::cout<<"goal_number: "<< goal_number <<std::endl;

 }

void NextWaypointCreator::process()
{
    ros::Rate loop_rate(hz);
    while(ros::ok())
    {
        if(have_recieved_path)
        {
            select_next_goal();
            next_waypoint.header.frame_id = "map";
            std::cout<<"next_waypoint :"<<next_waypoint.pose.position.x<<","<<next_waypoint.pose.position.y<<std::endl;
            pub_next_waypoint.publish(next_waypoint);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}

int main (int argc,char **argv)
{
    ros::init(argc, argv, "next_waypoint_creator");
    NextWaypointCreator next_waypoint_creator;
    next_waypoint_creator.process();
    return 0;
}