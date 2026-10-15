# depthai_oak_d

OAK-D (OAK-D-S2) 用の ROS 2 Jazzy ブリングアップパッケージです。
`cub_bringup/src/oak-d_s2_record.py` と同じ処理を、公式ドライバ
[depthai_ros_driver_v3](https://docs.luxonis.com/software-v3/depthai/ros/driver/) で起動します。

| 元スクリプト (depthai v3 Python) | 本パッケージ |
|---|---|
| `Camera(CAM_A)` → 1280x720 NV12 | `rgb.i_board_socket_id: 0`, `i_width/i_height` |
| `VideoEncoder` (H264_MAIN) | `rgb.i_low_bandwidth: true`, `i_low_bandwidth_profile: 2` |
| `RecordVideo` (.mp4) | `ros2 bag record` (mcap 形式) |

> フォルダ名は `depthai_oak-d` ですが、ROS のパッケージ名は `-` を使えないため **`depthai_oak_d`** です。

## 依存パッケージ

```bash
sudo apt install ros-jazzy-depthai-ros-v3 ros-jazzy-ffmpeg-image-transport-msgs
# RViz / rqt で H264 映像を表示する場合 (デコーダプラグイン)
sudo apt install ros-jazzy-ffmpeg-image-transport
```

## ビルド

```bash
cd ~/colcon_ws
colcon build --packages-select depthai_oak_d
source install/setup.bash
```

## 使い方

```bash
# RGB (H264) を配信
ros2 launch depthai_oak_d oak_d.launch.py

# RGB を配信しつつ rosbag に記録 (./test_video/ に保存)
ros2 launch depthai_oak_d oak_d.launch.py record:=true output:=test_video

# RGB + Depth
ros2 launch depthai_oak_d oak_d.launch.py mode:=rgbd record:=true
```

### Launch 引数

| 引数 | デフォルト | 説明 |
|---|---|---|
| `mode` | `rgb` | `rgb` または `rgbd`。`config/<mode>_h264.yaml` を読み込む |
| `params_file` | (空) | 任意のパラメータファイルで上書き |
| `record` | `false` | `true` で `ros2 bag record` を同時起動 |
| `output` | `test_video` | rosbag の出力ディレクトリ |
| `name` | `oak` | ノード名 / トピックの接頭辞 / TF の接頭辞 |
| `camera_model` | `OAK-D-S2` | URDF のモデル名 |
| `parent_frame` | `oak_parent_frame` | カメラ TF の親フレーム |
| `use_rviz` | `false` | RViz を起動 |

## 出力トピック

| トピック | 型 | 備考 |
|---|---|---|
| `/oak/rgb/image_raw/compressed` | `ffmpeg_image_transport_msgs/FFMPEGPacket` | H264 ビットストリーム (30 Hz) |
| `/oak/rgb/camera_info` | `sensor_msgs/CameraInfo` | |
| `/oak/stereo/image_raw` | `sensor_msgs/Image` (16UC1, mm) | `mode:=rgbd` のみ。RGB に位置合わせ済み |

> [!IMPORTANT]
> H264 はドライバ内で `sensor_msgs/Image` に戻せないため、`i_publish_compressed: true` が必須です。
> これがないと**トピックはできても画像が配信されません**。
> 非圧縮画像が必要な場合は `ffmpeg_image_transport` でデコードするか、
> `i_low_bandwidth: false` にしてください (USB 帯域を多く使います)。

## 設定ファイル

- `config/rgb_h264.yaml` : RGB のみ。元スクリプトと同じ構成
- `config/rgbd_h264.yaml` : RGB (H264) + Depth (非圧縮)。Depth は H264 だと劣化するため非圧縮で出力

`i_low_bandwidth_profile` の値: `0` H264_BASELINE / `1` H264_HIGH / `2` H264_MAIN / `3` H265_MAIN / `4` MJPEG

## 記録データの確認・変換

```bash
ros2 bag info test_video
ros2 bag play test_video
```

mp4 が必要な場合は、`FFMPEGPacket` の `data` を順番に書き出すと H264 ストリームになり、
`ffmpeg -i out.h264 -c copy out.mp4` で変換できます (変換スクリプトは未同梱)。
