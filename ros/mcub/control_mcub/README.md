# control_mcub
mcubをRaspi経由で操縦するためのパッケージ群

# Requirement
・mcubを上から見て左をID1, 右をID2に設定していること
  IDの変更は下記を参照
  https://github.com/CoderDojoMusashikosugi/Cub_ROS/tree/feature-add-mcub-environment/support_tools/dynamixel

# Submoduleの取得
```bash
git submodule update --init --recursive
```
# build方法
コンテナ起動(run.sh実行)後に下記コマンドを実行
```bash
cd colcon_ws
colcon build
bashrc
```

# 使用方法
### コンテナ内でDynamixelを制御するノードを起動する
```bash
ros2 run control_mcub control_mcub_moter_node
```
### キーボードの矢印キーで制御するteleopノードを起動
新たなターミナルを別途起動し、コンテナ内に入った(run.sh実行）後、下記コマンドを実行
```bash
ros2 run control_mcub mcub_teleop_key
```
矢印キーで移動。スペースで停止。
