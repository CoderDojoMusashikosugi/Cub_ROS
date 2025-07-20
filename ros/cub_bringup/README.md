# cub_bringup
Cub3/mCubの起動用launchファイルと、その他便利ツールの置き場

`cub_bringup cub.launch.py` とか実行したらCub3の全てのプロセスやデバッグツールが起動して欲しいなと思っている。



# glimの起動方法
コンテナ内で下記コマンドで実行
``` bash
ros2 run glim_ros glim_rosbag  {rosbagファイル} --ros-args -p config_path:=/home/cub/colcon_ws/src/cub/cub_bringup/params/glim_params/
```
細かい設定は下記Link参照  
[glim/quickstart](https://koide3.github.io/glim/quickstart.html)  
[glim/parameters](https://koide3.github.io/glim/parameters.html)

環境に応じて下記ファイルのimu, pointcloudのトピック名を修正してください。  
[config_ros.json](/ros/cub_bringup/params/glim_params/config_ros.json)