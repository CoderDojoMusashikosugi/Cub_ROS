# Cub_ROS
## Clone
```
$ git clone --recursive https://github.com/CoderDojoMusashikosugi/Cub_ROS.git
```

## install
```
$ sudo apt install libpcap-dev libyaml-cpp-dev
```

## build
```
$ cd ws
$ catkin_make -DCMAKE_BUILD_TYPE=Release
$ source devel/setup.bash
```

## launch
```
$ roslaunch hesai_lidar hesai_lidar.launch lidar_type:="PandarXT-32" frame_id:="PandarXT-32"
```

## topic name
```
/hesai/pandar
```
