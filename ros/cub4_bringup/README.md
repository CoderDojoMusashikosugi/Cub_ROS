# cub4_bringup

## MID360 + 内蔵IMUのrosbagからGLIM地図を生成

設定一式は[`config/glim_params_mid360`](config/glim_params_mid360/README.md)です。
MID360の点群と内蔵IMU、サブアンテナのRTK FIXを使用します。
GNSS原点・ジオイド補正・二段階処理はVelodyne版と共通です。

```bash
./docker/run_in_container.sh 'cbs cub4_bringup'
./docker/run_in_container.sh 'ros2 launch cub4_bringup glim_mid360.launch.py rosbag:=/home/cub/rosbag2_2026_10_17-15_09_06_no_images'
```

出力は`~/maps/<bag名>_glim_mid360_<日時>/`です。先頭180秒だけ確認する場合は
`playback_duration:=180`、GNSSを使わない場合は`enable_gnss:=false`を追加してください。

## Velodyne + IMU の rosbag から GLIM 地図を生成

CUDA対応の `glim_ros` と MCAP storage plugin を導入したコンテナで実行します。
設定は [`config/glim_params_velodyne`](config/glim_params_velodyne/README.md) にあります。
点群とIMUは`/velodyne_points` と `/imu/data_raw`を使用します。
既定ではサブアンテナRTK FIXも統合し、`enable_gnss:=false`で無効化できます。

ホストのプロジェクトルートから:

```bash
./docker/run_in_container.sh 'cbs cub4_bringup'
./docker/run_in_container.sh 'ros2 launch cub4_bringup glim_velodyne.launch.py rosbag:=/home/cub/rosbag2_2026_10_17-15_09_06_no_images'
```

依存パッケージも未ビルドの場合は、最初に以下を実行してください（GLIMはコンテナに導入済みのものを使います）。

```bash
./docker/run_in_container.sh 'cd /home/cub/colcon_ws && colcon build --symlink-install --packages-up-to cub4_bringup --packages-ignore glim glim_ros'
```

処理完了後、`/home/cub/maps/<bag名>_glim_velodyne_<実行日時>/` にGLIM形式の地図を保存して終了します。
ホストでは `docker/home/maps/` に対応します。出力先を固定する場合は `dump_path:=/home/cub/maps/任意の空ディレクトリ` を追加してください。既存データのある出力先はエラーになります。
保存内容は `graph.bin`、`values.bin`、サブマップ、軌跡（`traj_lidar.txt` 等）、使用したJSON設定です。
点群PLYとして書き出す場合は、保存したディレクトリをoffline viewerで開いてExportを使用します。

```bash
./docker/run_in_container.sh 'ros2 run glim_ros offline_viewer /home/cub/maps/生成されたディレクトリ'
```

GUI環境なしで地図生成が可能です。offline viewerでの表示・PLY出力にはGUI環境が必要です。
bagはGLIMが直接読み込むため、別途 `ros2 bag play` やセンサドライバ、ロボット制御ノードを起動する必要はありません。

| launch引数 | 既定値 | 説明 |
|---|---|---|
| `rosbag` | 必須 | コンテナ内のbagディレクトリまたはMCAPファイル |
| `dump_path` | 自動生成 | 地図の出力先（空ディレクトリ） |
| `config_path` | パッケージ内 `config/glim_params_velodyne` | GLIM設定ディレクトリ |
| `start_offset` | `0.0` | 読み飛ばす秒数 |
| `playback_duration` | `0.0` | 読み飛ばした後の処理秒数（0は最後まで） |

例えば先頭60秒のみを確認するには `playback_duration:=60.0` を追加します。
このbagに合わせてVelodyne時刻に-1.0秒の補正を設定しています。詳細は設定ディレクトリのREADMEを参照してください。

## spresense_imu_node

Spresense からシリアル通信経由で送信される 6 軸 IMU（CXD5602PWBIMU）および外部 GNSS（UM982 等）による高精度時刻同期パケットを受信し、ROS2 トピックとして配信するノードです。

### 起動方法

```bash
ros2 run cub4_bringup spresense_imu_node --ros-args -p serial_port:=/dev/ttyMULIMU -p baud_rate:=230400
```

### パラメータ

| パラメータ名 | 型 | デフォルト値 | 説明 |
|---|---|---|---|
| `serial_port` | string | `"/dev/ttyMULIMU"` | シリアルポートのデバイスパス |
| `baud_rate` | int | `230400` | 通信ボーレート (bps) |
| `frame_id` | string | `"imu_link"` | IMU メッセージの frame_id |
| `publish_temperature` | bool | `true` | 温度トピックを配信するかどうか |
| `use_gnss_time` | bool | `true` | GNSS 時刻同期が有効な場合に GNSS UTC タイムスタンプを使用するかどうか |
| `data_timeout_sec` | double | `5.0` | シリアルデータ途絶タイムアウト秒数（Spresense 起動待ち時間含む） |

### 配信トピック

- `imu/data_raw` (`sensor_msgs/msg/Imu`): 6軸 IMU データ (約240Hz)
  - `angular_velocity`: 角速度 [rad/s] (Spresense CXD5602PWBIMU ドライバ出力)
  - `linear_acceleration`: 加速度 [m/s^2] (Spresense CXD5602PWBIMU ドライバ出力)
- `imu/temperature` (`sensor_msgs/msg/Temperature`): センサ温度データ [℃]

---

### 時刻同期ステータスビット列フォーマット仕様

ノードが Spresense から受信するバイナリパケットには、1 バイトのステータスフラグ（`status`）が含まれています。  
時刻同期状態や割込キャプチャ状態に変化があった際、ログメッセージの先頭に 6 文字のビット列 `[xxxxxx]` 形式で状態が表示されます。

各文字は **MSB（Bit 5）から LSB（Bit 0）** の順に並んでおり、該当ビットが立っている場合は `x`、立っていない場合は `-` となります。

| 桁（左から） | 対象ビット | フラグ定義名 | 説明 |
|:---:|:---:|:---|:---|
| 1 文字目 | Bit 5 | `STATUS_DRDY_TIMED` | IMU DRDY 割込エッジ（D18）による正確なタイムスタンプ取得が有効 |
| 2 文字目 | Bit 4 | `STATUS_SYNC_WITHIN_1H` | 最終 GNSS 同期が 1 時間以内 |
| 3 文字目 | Bit 3 | `STATUS_SYNC_WITHIN_1M` | 最終 GNSS 同期が 1 分以内 |
| 4 文字目 | Bit 2 | `STATUS_SYNC_WITHIN_2S` | 最終 GNSS 同期が 2 秒以内（新鮮な同期状態。有効時は GNSS 時刻を使用） |
| 5 文字目 | Bit 1 | `STATUS_SYNC_EVER` | 起動後に 1 回以上 GNSS 同期に成功した履歴あり |
| 6 文字目 | Bit 0 | `STATUS_CLOCK_SYNCHRONIZED` | クロックが現在 GNSS と同期中 |

#### 表示パターン例

- **`[xxxxxx]`** (値: `0x3F`)  
  全ビット正常。DRDY 割込エッジ取得が有効かつ GNSS 時刻同期が正常にロックしている状態。
- **`[------]`** (値: `0x00`)  
  全ビット無効。起動直後や未同期の状態。
- **`[x-----]`** (値: `0x20`)  
  DRDY 割込エッジ取得のみ有効で、GNSS 時刻同期は未確立の状態。
- **`[-x----]`** (値: `0x10`)  
  1 時間以内同期フラグのみ有効。
- **`[-----x]`** (値: `0x01`)  
  クロック同期フラグのみ有効。
- **`[xx-xxx]`** (値: `0x3B` など)  
  GNSS 同期パケットの途絶（2 秒超過）などにより同期ロックが外れた状態。

#### ログ出力例

```text
[INFO] [spresense_imu_node]: [------] Waiting for GNSS sync. Using ROS system time.
[INFO] [spresense_imu_node]: [x-----] IMU DRDY edge capture (D18) ACTIVE. Timestamps locked to DRDY interrupt edge.
[INFO] [spresense_imu_node]: [xxxxxx] GNSS time synchronization locked! Using GNSS UTC timestamps.
[WARN] [spresense_imu_node]: [xx-xxx] GNSS time sync lost. Falling back to ROS system time.
```
