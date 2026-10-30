# ekf_localizer_ros2

LiDAR（NDT / fast_gicp）、IMU、車輪オドメトリ、および高精度 GNSS（RTK-GPS）を拡張カルマンフィルタ（EKF）により統合する自己位置推定パッケージです。

## Environment
- ROS 2 Jazzy / Humble
- CUDA & GPU環境（fast_gicp / NDTCuda）
- GeographicLib

---

## Topic Interface

### map_matcher_node (NDT 点群マッチング)
#### Subscriber
- `/velodyne_points` (`sensor_msgs/msg/PointCloud2`): LiDAR点群データ
- `/test/ekf_pose` (`geometry_msgs/msg/PoseStamped`): NDT探索の初期推定値（init_guess）
#### Publisher
- `/map_out` (`sensor_msgs/msg/PointCloud2`): 読み込んだ点群地図
- `/test/ndt_pose` (`geometry_msgs/msg/PoseStamped`): NDT推定位置（ジャンプガード通過後のみ）
- `/ndt_pc` (`sensor_msgs/msg/PointCloud2`): マッチング後点群

### ekf_localizer_node (拡張カルマンフィルタ)
#### Subscriber
- `/odom` (`nav_msgs/msg/Odometry`): 車輪オドメトリ
- `/imu/data_raw` (`sensor_msgs/msg/Imu`): IMUデータ（TFにより自動でbase_link系へ変換）
- `/test/ndt_pose` (`geometry_msgs/msg/PoseStamped`): NDT位置測定値
- `/gps_pose` (`geometry_msgs/msg/PoseWithCovarianceStamped`): 局所平面直交座標系（map系）に変換されたGPS位置
- `/um982_node/fix` (`sensor_msgs/msg/NavSatFix`): RTK-FIXステータス検証用トピック
- `/initialpose` (`geometry_msgs/msg/PoseWithCovarianceStamped`): RViz等からの初期位置指定
#### Publisher
- `/test/ekf_pose` (`geometry_msgs/msg/PoseStamped`): 推定自己位置（map系）

### gps_updater_node (GNSS 座標変換)
#### Subscriber
- `/um982_node/fix` (`sensor_msgs/msg/NavSatFix`): GNSS 受信機データ（UM982等）
- `/initialpose` (`geometry_msgs/msg/PoseWithCovarianceStamped`): 原点リセット用
#### Publisher
- `/gps_pose` (`geometry_msgs/msg/PoseWithCovarianceStamped`): mapフレームでの位置

---

## 主要な安全・安定化機能

### 1. NDT ジャンプガード機能 (`map_matcher.cpp`)
幾何特徴が薄い場所や旋回時に Levenberg-Marquardt（LM）最適化が局所解に飛び出した場合、初期推定値（`init_guess`）からの乖離量をチェックし、異常値を EKF に渡す前に水際で破棄します。
- `MAX_TRANSLATION_JUMP`: 許容並進変位（デフォルト: `1.0` [m]）
- `MAX_ROTATION_JUMP_DEG`: 許容回転変位（デフォルト: `30.0` [deg]）
- `MATCHING_SCORE_TH`: フィットネススコア上限（`0.25`）

### 2. URDF (TF) 準拠の IMU 座標変換 & フォールバック機能 (`ekf.cpp`)
IMU が上下逆さま（`cub4.urdf` の `rpy="3.14 0 0"`）や任意の角度でマウントされている場合でも、`base_link` $\leftarrow$ `imu_link` の TF 変換を取得して角速度・加速度ベクトルを自動で `base_link` 座標系に回転変換します。
- `use_imu_tf: true`: TF による変換を有効化。
- `imu_yaw_sign: -1.0`: TF がまだ受信できていない場合（rosbag 再生時等）の安全フォールバック、または `use_imu_tf: false` 時の直接符号倍率。TF が未受信でも EKF の角速度積算が停止（フリーズ）せず動作を継続します。
- `ekf_locali.launch.py` には `robot_state_publisher` が内包されており、単体起動や rosbag 再生時でも `cub4.urdf` に基づく TF が自動配信されます。

### 3. RTK-FIX 限定 GPS リスポーン機能 (`ekf.cpp`)
NDT の途絶時やドリフト時に、真に高精度な RTK-FIX 状態であるときのみ GPS 座標へのリスポーン（位置復旧）を許可します。
- `require_rtk_fix_for_respawn: true`: `/um982_node/fix` のステータスが `STATUS_GBAS_FIX`（RTK-FIX）であることを要求
- `gps_high_confidence_threshold: 0.1` [m]: RTK-Float や単独測位のブレによる誤テレポートを遮断

---

## Install and Build

```bash
sudo apt update
sudo apt install libgeographic-dev geographiclib-tools

# ビルド
cd ~/colcon_ws
colcon build --packages-select ekf_localizer
source install/setup.bash
```

---

## Configuration

### 1. `ekf.yaml`
```yaml
ekf_localizer_node:
  ros__parameters:
    base_link_frame_id: "base_link"
    map_frame_id: "map"
    odom_frame_id: "odom"

    # IMU設定 (TFによる自動変換 & フォールバック)
    use_imu_tf: true
    imu_yaw_sign: -1.0                   # TF未受信時またはuse_imu_tf: false時のヨー角速度符号反転
    imu_topic_name: "/imu/data_raw"

    # GNSS設定 (RTK-FIX限定リスポーン)
    navsat_topic_name: "/um982_node/fix"
    require_rtk_fix_for_respawn: true
    gps_high_confidence_threshold: 0.1   # [m] RTK-FIX (数cm) のみ許可
    gps_respawn_enable: true             # リスポーン機能の有効化
    gps_respawn_distance_threshold: 3.0  # [m] 乖離閾値

    # 初期位置設定
    INIT_X: 0.0
    INIT_Y: 0.0
    INIT_Z: 0.0
    INIT_YAW: 0.0
```

### 2. `map_matcher.yaml`
```yaml
map_matcher_node:
  ros__parameters:
    pcd_file_path: "/home/cub/maps/map_tc25_gnss_0_2.pcd"

    # NDT最適化・ジャンプガード
    MATCHING_SCORE_TH: 0.25
    STEP_SIZE: 0.05
    TRANS_EPSILON: 0.01
    MAX_TRANSLATION_JUMP: 1.0     # [m]
    MAX_ROTATION_JUMP_DEG: 30.0   # [deg]
```

### 3. `gps_updater.yaml`
```yaml
gps_updater:
  ros__parameters:
    gps_topic_name: "/um982_node/fix"
    gps_pose_topic_name: "/gps_pose"
    map_frame_id: "map"

    use_manual_origin: true
    gps_origin_lat: 36.082628231
    gps_origin_lon: 140.076144739
    gps_origin_alt: 67.078
```

---

## Launch

```bash
ros2 launch ekf_localizer ekf_locali.launch.py
```
