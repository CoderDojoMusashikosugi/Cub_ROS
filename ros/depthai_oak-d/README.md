# depthai_oak_d

OAK-D (OAK-D-S2) 用の ROS 2 Jazzy ブリングアップパッケージです。
公式ドライバ [depthai_ros_driver_v3](https://docs.luxonis.com/software-v3/depthai/ros/driver/) を使い、
CAM_A (RGB) の映像を **1280x720 の `sensor_msgs/Image`** として配信します。
記録は別途 `ros2 bag record` で行います。

> フォルダ名は `depthai_oak-d` ですが、ROS のパッケージ名は `-` を使えないため **`depthai_oak_d`** です。

## 依存パッケージ

```bash
sudo apt install ros-jazzy-depthai-ros-v3
```

## ビルド

```bash
cd ~/colcon_ws
colcon build --packages-select depthai_oak_d
source install/setup.bash
```

## 使い方

```bash
# RGB を配信
ros2 launch depthai_oak_d oak_d.launch.py

# RGB + Depth を配信
ros2 launch depthai_oak_d oak_d.launch.py mode:=rgbd

# RViz で確認
ros2 launch depthai_oak_d oak_d.launch.py use_rviz:=true
```

### 記録 (別ターミナル)

```bash
# RGB
ros2 bag record -s mcap -o test_video /oak/rgb/image_raw /oak/rgb/camera_info /tf_static

# RGB + Depth
ros2 bag record -s mcap -o test_rgbd /oak/rgb/image_raw /oak/rgb/camera_info \
  /oak/stereo/image_raw /oak/stereo/camera_info /tf_static
```

> [!NOTE]
> 非圧縮の 1280x720 BGR8 画像を 30 fps で記録すると、約 80 MB/s になります。ディスクの空き容量に注意してください。

### Launch 引数

| 引数 | デフォルト | 説明 |
|---|---|---|
| `mode` | `rgb` | `rgb` または `rgbd`。`config/<mode>.yaml` を読み込む |
| `params_file` | (空) | 任意のパラメータファイルで上書き |
| `name` | `oak` | ノード名 / トピックの接頭辞 / TF の接頭辞 |
| `camera_model` | `OAK-D-S2` | URDF のモデル名 |
| `parent_frame` | `oak_parent_frame` | カメラ TF の親フレーム |
| `use_rviz` | `false` | RViz を起動 |

## 出力トピック

| トピック | 型 | 備考 |
|---|---|---|
| `/oak/rgb/image_raw` | `sensor_msgs/Image` | 1280x720, 30 fps |
| `/oak/rgb/camera_info` | `sensor_msgs/CameraInfo` | |
| `/oak/stereo/image_raw` | `sensor_msgs/Image` (16UC1, mm) | `mode:=rgbd` のみ。RGB に位置合わせ済み |
| `/oak/stereo/camera_info` | `sensor_msgs/CameraInfo` | `mode:=rgbd` のみ |

`image_transport` の `compressed` などのトピックも自動で作られます。

## 設定ファイル

- `config/rgb.yaml` : RGB のみ (CAM_A, 1280x720, 30 fps)
- `config/rgbd.yaml` : RGB + Depth (Depth は RGB に位置合わせ)

解像度・fps は `rgb.i_width` / `rgb.i_height` / `rgb.i_fps` で変更できます。
