# ekf_localizer_ros2

## Overview
- ...

## Environment
- ROS2 humble
- ...

### NDT topics
#### Subscirber
- /velodyne_points(sensor_msgs/msg/PointCloud2)
#### Publisher
- /map_out(sensor_msgs/msg/PointCloud2)
- /test/ndt_pose(geometry_msgs/msg/PoseStamped)
- /ndt_pc(sensor_msgs/msg/PointCloud2)
### EKF topics
#### Subscirber
- /odom(nav_msgs/msg/Odometry)
- /bno055/imu(sensor_msgs/msg/IMU)
- /test/ndt_pose(geometry_msgs/msg/PoseStamped)
#### Publisher
- /test/ekf_pose(geometry_msgs/msg/PoseStamped)

### GPS updater topics
#### Subscirber
- /fix(sensor_msgs::msg::NavSatFix)
#### Publisher
- /gps_pose(geometry_msgs::msg::PoseWithCovarianceStamped)

## Install and Build
``` bash
sudo apt update
sudo apt install libgeographic-dev geographiclib-tools
# clone repository
git clone https://github.com/UNI-shibuya-taku/ekf_localizer_ros2.git

# build
cd ~/colcon_ws
colcon build
```

## How to use

### Configuration

1. Edit the ekf.yaml files:
   ```bash
   cd ekf_localizer_ros2/config
   vim ekf.yaml

    # map原点と実際のスタート位置に合わせて調整する(x,y,z,yaw)
    # 初期位置設定
    INIT_X: 0.0
    INIT_Y: 0.0
    INIT_Z: 0.0 
    INIT_YAW: 0.0

    # NDTによる推定位置が今までのEKFの推定位置から閾値以上離れていたらNDTの更新を行わない
    TH_MAHALANOBIS: 3.0

    # EKFの推定の分散が閾値を超えたら時観測更新を行う
    TH_COVARIANCE: 1.0 # 変更不要
    TH_POSE_COVARIANCE: 0.4 # 位置の分散の閾値
    TH_DIRECTION_COVARIANCE: 0.2 # 方位の分散の閾値
    GPS_MEASUREMENT_ENABLE: true # true: GPSによる観測更新を行う false: 行わない
   ```

1. Edit the map_matcher.yaml files:
   ```bash
   cd ekf_localizer_ros2/config
   vim map_matcher.yaml

    # pcdフォルダにmapデータを格納した後、mapデータのpathをmap_matcher.yamlに記載する
    pcd_file_path: /home/cub/colcon_ws/src/cub/ekf_localizer/pcd/mapkakunin_seg.pcd
    # NDTのボクセルサイズ(マシンスペックに合わせて調整)
    VOXEL_SIZE: 0.3
    VOXEL_SIZE_MAP: 0.2
   ```

2. Edit the launch file to update the file paths:
   ```bash
   cd ekf_localizer_ros2
   vim ekf.yaml
   vim ekf_locali.launch.py
   ```

   ```python
   # EKFパラメータ
    params_ekf = os.path.join(
        get_package_share_directory('ekf_localizer'),  # パッケージ名
        'config',  # ディレクトリ名
        'ekf',  # サブディレクトリ
        # 'ekf.yaml'  # ファイル名
        'ekf.yaml'  # ①EKFのyamlファイルを指定
    )
    # スキャンマッチングのパラメータ
    params_map_matcher = os.path.join(
        get_package_share_directory('ekf_localizer'),  # パッケージ名
        'config',  # ディレクトリ名
        'map_matcher',  # サブディレクトリ
        # 'map_matcher.yaml'  # ファイル名
        'map.yaml'  # ②スキャンマッチングのファイルを指定
    )
   ```


### RUN
``` bash
# run
ros2 launch ekf_localizer ekf_locali.launch.py
```