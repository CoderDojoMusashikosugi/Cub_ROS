#include "nav_manager/next_waypoint_creator.h"

NextWaypointCreator::NextWaypointCreator():private_nh("~")
{
    //parameter
    private_nh.param("border_distance", border_distance,{2.0});
    //subscriber
    sub_global_path = nh.subscribe("/global_path/path",10,&NextWaypointCreator::global_path_callback, this);
    sub_current_pose = nh.subscribe("/ekf_pose",10,&NextWaypointCreator::current_pose_callback, this);

    //publisher
    pub_next_waypoint = nh.advertise<geometry_msgs::PoseStamped>("/next_waypoint",1);
}

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