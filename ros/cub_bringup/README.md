# cub_bringup
Cub3/mCubの起動用launchファイルと、その他便利ツールの置き場

# ファイルの説明
- launch_at_boot.launch.py
  - Linuxが立ち上がってから落とすまでずっと起動しっぱなし想定のプログラムを纏めたもの。
  - `scripts/mcub_autostart_settings.sh`のように自動起動に設定してしまっても良い。
  - 通信する系のROSノードで、一度落とすとLinux再起動しないと再接続できないものがあった時に発生した。
  - 今は主に手動走行までが自動起動するために使われている。
- localization.launch.py
  - Localizationをスタートさせる。
- navigation.launch.py
  - Navigationをスタートさせる。
  - 内部でlocalization.launch.pyを呼んでる。
- mapping.launch.py
  - Mappingをスタートさせる。
  - 機体本体でマッピングするのはmcubだけかも。
- common.launch.py
  - mappingでもlocalizationでもnavigationでも共通して使うノードを置く。オドメトリのリセット&スタートなど。
- 2d/3d_mapping.launch.py
  - mapping.launch.pyの中身。
- rosbag.launch.py
  - (今の所)Cub3のセンサデータを記録開始する。

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