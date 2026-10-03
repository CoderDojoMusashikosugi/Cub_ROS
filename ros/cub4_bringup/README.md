# cub4_bringup

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
