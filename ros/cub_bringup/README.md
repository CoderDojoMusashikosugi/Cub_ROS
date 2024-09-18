# Cub_bringup

まず、cub2かmcub、どちらをbringupするかをtarget.envに書き記す。

```
CUB_TARGET=cub2
#CUB_TARGET=mcub

```

初期状態では、cub2になっている。

mcubを動かすときはCUB_TARGETをmcubにする。

すでに起動してしまっている場合は

```
./stop.sh
```

をした後、target.envを設定し、

```
./run.sh
```

を行う。

## mcub_bringup

CUB_TARGET=mcubの場合

起動するノードは次の通り

- slam_toolbox
- navigation2　※nav2
- mcub_assy_description robot_state_publisher
- sliidar_ros2
- wheel_odometry
- micro_ros_agent

nav2とslam_toolbox、sllidarに関してはconfigファイルを作成する予定です。

現在のlaunchファイルの構成では、slamでmapをリアルタイムに生成し、そのmap情報を頼りにnav2で自律移動をしている状態です。

ラズパイから描画したい際は、別のコンソールから

```
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

を実行して、foxgloveから接続してください。

＜gif挿入予定＞

foxglove上から、goal_poseを指定したら動きます。
