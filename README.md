# Cub_ROS

## Environment
Linux : ubuntu20.04
ROS : melodic

## Clone
```
$ git clone --recursive https://github.com/CoderDojoMusashikosugi/Cub_ROS.git
```

## install
### rosdep
```
$ rosdep install -i --from-paths ws/src
```

### Hesai Lidar
```
$ sudo apt install libpcap-dev libyaml-cpp-dev
```
### rosserial
```
$ sudo apt install ros-noetic-rosserial-arduino
$ sudo apt install ros-noetic-rosserial
```

## build
```
$ cd ws
$ catkin_make -DCMAKE_BUILD_TYPE=Release
$ source devel/setup.bash
```

## launch
### Hesai Lidar
```
$ roslaunch hesai_lidar hesai_lidar.launch lidar_type:="PandarXT-32" frame_id:="PandarXT-32"
```
### rosserial
**/dev/ttyACM0** は各環境に合わせる
```
$ roscore
$ rosrun rosserial_python serial_node.py /dev/ttyACM0
```

## topic name
### Hesai Lidar
Subscribe
```
/hesai/pandar
```
### rosserial
Subscribe
```
/atom/odometry : nav_msgs/Odometry
```
Publish
```
/atom/odometry_reset : std_msgs/Empty
/atom/motor_control : geometry_msgs/Twist
```
