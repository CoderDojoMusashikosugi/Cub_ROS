#include "nav_manager/global_path_creator.h"
#include "nav_manager/waypoints.h"

GlobalPathCreator::GlobalPathCreator() :
	private_nh_("~")
{
    pub_path = n.advertise<nav_msgs::Path>("/global_path/path",1);
    pub_id = n.advertise<visualization_msgs::MarkerArray>("/global_path/marker/id",1);
    pub_waypoint = n.advertise<visualization_msgs::MarkerArray>("/global_path/marker/waypoint",1);
    std::cout << "set global_path_creator!!!!" << std::endl;

    load_waypoints();
    load_route();
    make_global_path();
}
void GlobalPathCreator::load_waypoints()
{
    if(!private_nh_.getParam("waypoints_list",waypoints_list_)){
        ROS_WARN("Can not load waypoints list");
        return;
    }
    ROS_ASSERT(waypoints_list_.getType() == XmlRpc::XmlRpcValue::TypeArray);
    std::cout << "waypoints_size: " << waypoints_list_.size() << std::endl;
    for(int i = 0; i < (int)waypoints_list_.size(); i++){
        if(!waypoints_list_[i]["id"].valid() || !waypoints_list_[i]["x"].valid() || !waypoints_list_[i]["y"].valid()){
            ROS_WARN("waypoints list is valid");
            return;
        }
        if(waypoints_list_[i]["id"].getType() == XmlRpc::XmlRpcValue::TypeInt && waypoints_list_[i]["x"].getType() == XmlRpc::XmlRpcValue::TypeDouble && waypoints_list_[i]["y"].getType() == XmlRpc::XmlRpcValue::TypeDouble){
            int id = static_cast<int>(waypoints_list_[i]["id"]);
            double x = static_cast<double>(waypoints_list_[i]["x"]);
            double y = static_cast<double>(waypoints_list_[i]["y"]);
            Waypoint waypoint(id, x, y);
            waypoints_.push_back(waypoint);
        }
    }
}
void GlobalPathCreator::load_route()
{
    std::cout << "load_route" << std::endl;
    if(!private_nh_.getParam("route_list", route_list_)){
        ROS_WARN("Can not load route list");
        return;
    }
    ROS_ASSERT(route_list_.getType() == XmlRpc::XmlRpcValue::TypeArray);
    for(int i = 0; i < (int)route_list_.size(); i++){
        if(!route_list_[i]["id"].valid()){
            ROS_WARN("route list is valid");
            return;
        }
        if(route_list_[i]["id"].getType() == XmlRpc::XmlRpcValue::TypeInt){
            int id = static_cast<int>(route_list_[i]["id"]);
            routes_.push_back(id);
        }
    }
}

void GlobalPathCreator::make_global_path()
{
    visualization_msgs::MarkerArray id_markers;
    visualization_msgs::MarkerArray waypoint_markers;

    // routes_[0]
    // waypoints_[0]
    int count_id = 0;
    for(int i = 0; i < routes_.size(); i++){
        int id = routes_[i];
        for(int j = 0; j < waypoints_.size(); j++){
            if(waypoints_[j].id == id){
                geometry_msgs::PoseStamped tmp_path_point;
                tmp_path_point.pose.position.x = waypoints_[j].x;
                tmp_path_point.pose.position.y = waypoints_[j].y;
                tmp_path_point.header.frame_id = "map"; 
                global_path.poses.push_back(tmp_path_point);
                std::cout << "route_id: " << id << std::endl;

                visualization_msgs::Marker id_mk;
                id_mk.header.frame_id = "map";
                id_mk.header.stamp = ros::Time::now();
                id_mk.id = count_id;
                id_mk.ns = "basic_shapes";
                std::ostringstream str;
                str << id;
                id_mk.text = str.str();
                id_mk.action = visualization_msgs::Marker::ADD;
                id_mk.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
                id_mk.lifetime = ros::Duration();
                id_mk.pose.position.x = waypoints_[j].x;
                id_mk.pose.position.y = waypoints_[j].y;
                id_mk.scale.z = 0.4;
                id_mk.pose.orientation.w = 1;
                id_mk.color.r = 1.0;
                id_mk.color.g = 1.0;
                id_mk.color.b = 1.0;
                id_mk.color.a = 0.7;
                id_markers.markers.push_back(id_mk);
                
                visualization_msgs::Marker waypoint_mk;
                waypoint_mk.header.frame_id = "map";
                waypoint_mk.header.stamp = ros::Time::now();
                waypoint_mk.id = id;
                waypoint_mk.action = visualization_msgs::Marker::ADD;
                waypoint_mk.type = visualization_msgs::Marker::CUBE;
                waypoint_mk.lifetime = ros::Duration();
                waypoint_mk.pose.position.x = waypoints_[j].x;
                waypoint_mk.pose.position.y = waypoints_[j].y;
                waypoint_mk.scale.x = 0.3;
                waypoint_mk.scale.y = 0.3;
                waypoint_mk.scale.z = 0.3;
                waypoint_mk.pose.orientation.w = 1;
                waypoint_mk.color.r = 1.0;
                waypoint_mk.color.g = 0.0;
                waypoint_mk.color.b = 0.0;
                waypoint_mk.color.a = 0.7;
                waypoint_markers.markers.push_back(waypoint_mk);

                count_id ++;
            }
        } 
    }
    global_path.header.frame_id = "map";
    // id_markers.header.frame_id = "map";
    // waypoint_markers.header.frame_id = "map";
    std::cout << "marker array_size: " << id_markers.markers.size() << " " << waypoint_markers.markers.size() << std::endl;
    int count = 0;
    while(1){
        pub_path.publish(global_path);
        pub_id.publish(id_markers);
        pub_waypoint.publish(waypoint_markers);
    }
}

void GlobalPathCreator::process()
{
    while(ros::ok()){
        // pub_path.publish(global_path);
        // ros::spinOnce();
    }
}
int main (int argc, char **argv)
{
  ros::init(argc,argv,"global_path_creator");
  GlobalPathCreator global_path_creator;
  global_path_creator.process();
  return 0;
}