# nav_manager
- simple_localmap_creator
- global_path_creator
- next_waypoint_creator

## Enviornment
- Ubuntu 20.04
- ROS noetic

## Install and Build

```
cd catkin_workspace/src
git clone https://github.com/CoderDojoMusashikosugi/Cub_ROS.git
cd ..
catkin_make
```

## Nodes
### localmap_creator
## Published topics
- /local_map(nav_msgs::OccupancyGrid)
- /local_map/expand(nav_msgs::OccupanceGrid)
## Subscribed topics
- /velodyne_obstacles(sensor_msgs::PointCloud2)
<!-- ## Parameters -->
## How to Use
```
roslaunch nav_manager simple_localmap_creator.launch
```

### global_path_creator
## Published topics
- /global_path/path(nav_msgs::Path)
- /global_path/id(visualization_msgs::MarkerArray)
- /global_path/waypoint(visualization_msgs::MarkerArray)

<!-- ## Subscribed topics -->
<!-- - / -->
<!-- ## Parameters -->
## How to Use
```
cd nav_manager/config
```
- edit waypoints_list.yaml & route_list.yaml
```
roslaunch nav_manager global_path_creator.launch
```

### next_waypoint_creator
## Published topics
- /next_waypoint(geometry_msgs::PoseStamped)

## Subscribed topics
- /global_path/path(nav_msgs::Path)
- /ekf_pose(geometry_msgs::PoseStamped)

<!-- ## Parameters -->
## How to Use
```
$ roslaunch nav_manager next_waypoint_creator.launch 
```