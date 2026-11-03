# Cub simulation interface

`simulation:=none`（既定）は実機、`simulation:=godot`はGodot接続を選びます。

```bash
ros2 launch cub_bringup simulation.launch.py simulation:=godot
```

このlaunchは実機の`launch_at_boot.launch.py`の代わりに起動します。`common.launch.py`は必要な場合だけ別途`simulation:=godot`を付けて起動します。実機起動は従来のlaunchを使います。`simulation:=none`のsimulation launchは案内を出して終了し、実機ドライバーを勝手に起動しません。

## インターフェース

- Godot → `/joy` (`sensor_msgs/msg/Joy`)：既存Linux DualSenseの8軸・14ボタン形式、20 Hz。入力元は接続した認識済みコントローラー、未接続時はキーボードへ自動切り替え。WASD/SpaceまたはコントローラーのL1/L2が押されている間だけ手動、無操作時は自律指令を使用。フォーカス喪失による手動停止は行いません。
- Godot → `/scan` (`sensor_msgs/msg/LaserScan`)：ホストで計算する360度・360本・10 Hzの簡易2D LiDAR。位置と`frame_id`は`cub4.urdf`の`base_to_velodyne`（`velodyne`、`base_link`から高さ0.2492 m）を使用。壁・箱・他のロボット・通行人・コーンのセンサー用形状を検出し、床・坂道・階段・凹凸地形を除外します。
- Godot → `/odom` (`nav_msgs/msg/Odometry`)：主ロボットのシミュレータ座標（Godot空間の真値姿勢・速度）をROS座標系（`odom -> base_link`）へ変換して20 Hzで配信。`header.stamp`は省略し、ROS側のrosbridgeがROS時刻を補完します。
- `/joy` → `cub_commander` → `/cmd_vel_atom` (`geometry_msgs/msg/Twist`) → Godot。
- 自律ノードは既存と同じ`/cmd_vel`へ送信。Godotは`linear.x`と`angular.z`だけを運動に使います。既存の`angular.y`の操作元フラグを回転へ適用しません。
- Godot URLの既定値は`ws://127.0.0.1:9090`。launchの`address`・`port`で待ち受けを変更できます。VNCのポート公開を変える場合はCompose側も合わせます。
- commanderとJoypadはコミット済みの実装を使用します。`/dev/input/js0`の機種判別で不明な場合にDualSense扱いとなる既存動作を利用し、ROS側にXboxが接続されている構成はGodotの送信形式と一致しません。
- commanderの接続判定はJoyの`header.stamp`から1秒、自律指令の受信途絶は3秒です。Godotが省略したJoyのheaderはrosbridgeがROS時刻で補完します。追加のタイムアウトパラメーターや停止維持機能は使用しません。

操作・導入手順は[Godot README](../../simulator/README.md#rosを通した主ロボットの操作)を参照してください。

## ノードごとの起動条件

`simulation`はROS 2標準のlaunch引数です。YAMLプロファイルや中間の機能フラグは使わず、各launch内でノードの起動条件を直接指定します。

```python
Node(
    package='wheel_odometry',
    executable='wheel_odometry_node',
    condition=IfCondition(EqualsSubstitution(LaunchConfiguration('simulation'), 'none')),
)
```

実機向けのwheel odometry・シリアルボタン・GNSS監視・オドメトリクリアは`none`で起動します。commanderとrosbridgeはsim用bringup内の`godot`条件で起動します。共有するノードには条件を付けません。診断トピックの設定ファイルは、Cub4のcommon内で`simulation`を見て選択します。`godot_diagnostics.yaml`は監視内容の設定で、ノードの有効／無効一覧ではありません。

ノードを追加・削除するときは、そのlaunch内で起動条件を決めるだけです。別のYAMLにノード一覧を追記する必要はありません。新しいシミュレーターに対応するときは、該当launchの`simulation`引数の`choices`と、必要なノードの条件・接続設定を更新します。現在の対応値は`none`と`godot`で、未知の値はROS launchの引数検証で拒否します。

### 共通ノード（URDF / robot_state_publisher 等）の起動

機体ターゲットは実機・シミュレーション共通で環境変数`CUB_TARGET`（未指定時は`cub4`）に従います。`use_sim_time`は独立した時計の選択で、今回のGodot接続は`false`です。

```bash
# 実機
ros2 launch cub_bringup common.launch.py simulation:=none

# Godotシミュレーション (Cub4)
ros2 launch cub_bringup common.launch.py simulation:=godot
```

### 自己位置推定の起動

`localization.launch.py simulation:=godot`は実機用EKFの代わりに`odom_to_tf`で`odom -> base_link`を出し、`map -> odom`の静的TFと合わせて`map -> odom -> base_link -> velodyne`を構成します（内部で`common.launch.py`も起動されます）。

```bash
ros2 launch cub_bringup localization.launch.py simulation:=godot
```

### 自律移動 (Nav2) の起動

`navigation.launch.py simulation:=godot`は実機と同じく`map:=<path>`引数で地図ファイルを受け取り、Godotの`/scan`と`/odom`を使ってNav2を動かします。Godot空間に対応する地図は`cub_simulation/maps/cub_playground_v1/map.yaml`にあります（内部で`localization.launch.py`も起動されます）。

```bash
ros2 launch cub_bringup navigation.launch.py simulation:=godot map:=/home/cub/colcon_ws/src/cub/cub_simulation/maps/cub_playground_v1/map.yaml
```

## テスト

```bash
python3 -m pytest /home/cub/colcon_ws/src/cub/cub_simulation/test -q
```

実launchのノードごとの条件判定・コミット済みC++ commanderの速度生成、手動ゼロ指令、手動解除後の自律指令受け付け、Joyの1秒途絶での自律復帰、自律指令の3秒途絶での停止を検証します。commanderのテストはROS_DOMAIN_ID=172を使い、作成したプロセスを終了時に片付けます。

Godot側のJoy変換・入力機器の自動選択・ROS未接続時の停止・簡易LiDARのテスト:

```bash
/Applications/Godot.app/Contents/MacOS/Godot --headless --path simulator --script res://tests/ros_control_test.gd
/Applications/Godot.app/Contents/MacOS/Godot --headless --path simulator --script res://tests/lidar_test.gd
```

実際の往復接続は、sim用launch起動後、上記コマンドの末尾へ`-- --integration --ros-url ws://127.0.0.1:9090`を追加します。W/Aの移動、Spaceによる手動停止とWASD優先、無操作時の自律復帰、フォーカス・カーソル解放に依存しない中継、切断・再接続を確認します。コントローラーの接続状態・SDL入力は模擬して自動選択と自律・手動の切り替えを検証します。物理コントローラーのUSB/Bluetooth接続は別途実機で確認してください。
