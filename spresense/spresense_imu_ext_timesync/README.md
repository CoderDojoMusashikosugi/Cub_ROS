# Spresense IMU + 外部GNSS時刻同期 (spresense_imu_ext_timesync)

Spresense 内蔵 6 軸 IMU（CXD5602PWBIMU）のデータを、外部 GNSS モジュール（UM982 など）からの 1PPS および NMEA 信号を用いて高精度 UTC 時刻同期し、バイナリパケットでホスト PC（ROS2）へストリーミング送信するスケッチです。

受信側の ROS2 ノードは `ros/cub4_bringup` パッケージ内の `spresense_imu_node` を使用します。

---

## 主な機能

1. **6 軸 IMU バイナリストリーミング (240 Hz)**
   - 加速度・角速度・温度データをハードウェア FIFO（閾値 4）経由で低遅延読み出し
   - 固定長バイナリフレーム（ヘッダ 4B + ペイロード 41B + チェックサム 1B = 46B）で高速送信

2. **外部 GNSS モジュールによる高精度 UTC 時刻同期**
   - **1PPS 入力（D3 ピン）**: 立ち上がりエッジのマイクロ秒時刻（`micros()`）を割り込みキャプチャ
   - **NMEA 受信（Serial2 / 9600 bps）**: `TinyGPSPlus` で PPS 直後の NMEA センテンスから UTC 日時を取得
   - マイクロ秒精度の絶対 UTC タイムスタンプ（`utc_timestamp_us`）をパケットに付与

3. **時刻同期ステータスフラグ (`status`)**
   - パケット内の `status` バイト（ビットフラグ）で同期状況を通知：
     - `0x01` (Bit 0): 現在時刻同期が有効
     - `0x02` (Bit 1): 起動後に同期履歴あり
     - `0x04` (Bit 2): 2秒以内に時刻同期パケットを受信（同期ロック中）
     - `0x08` (Bit 3): 1分以内に時刻同期パケットを受信
     - `0x10` (Bit 4): 1時間以内に時刻同期パケットを受信

---

## ハードウェア配線 (UM982 と Spresense 拡張ボード)

Spresense 拡張ボードの I/O 電圧ジャンパ **JP1 を必ず 3.3V 側に設定** してください。

| Spresense 拡張ボード | UM982 モジュール | 用途 |
|---|---|---|
| **GND** | GND | 共通 GND |
| **D0 (Serial2 RX)** | TXD (COM1/2/3) | NMEA / `#EVENTFLAGA` メッセージ受信 |
| **D1 (Serial2 TX)** | RXD (同じ COM) | UM982 初期化設定コマンド送信 |
| **D3** | 1PPS | 1秒周期のパルス信号（時刻同期） |
| **D4** | EVENT / EVE | （任意）イベント同時入力ピン（モーメンタリボタン接続） |

---

## 通信仕様 (Serial: 230400 bps)

- **パケット構成 (46 バイト固定長)**:
  - **ヘッダ (4 bytes)**: `0xAA, 0xBB, 0xCC, 0xDD`
  - **ペイロード (`SyncedIMUData`, 41 bytes)**:
    - `utc_timestamp_us` (`uint64_t`, 8B): 同期 UTC マイクロ秒タイムスタンプ（未同期時は 0）
    - `sensor_timestamp` (`uint32_t`, 4B): CXD5602PWBIMU 19.2MHz チックカウント
    - `temp` (`float`, 4B): 温度 [℃]
    - `gx, gy, gz` (`float` × 3, 12B): 角速度 [rad/s]
    - `ax, ay, az` (`float` × 3, 12B): 加速度 [G]
    - `status` (`uint8_t`, 1B): 時刻同期ステータスフラグ
  - **チェックサム (1 byte)**: ペイロード全41バイトの XOR 値

---

## ROS2 受信ノード (`cub4_bringup`)

`ros/cub4_bringup` パッケージ内の `spresense_imu_node` を起動して受信します。

### 起動例
```bash
ros2 run cub4_bringup spresense_imu_node --ros-args -p serial_port:=/dev/ttyMULIMU -p baud_rate:=230400
```

### 配信トピック
- `/imu/data_raw` (`sensor_msgs/msg/Imu`):
  - `header.stamp`: 2秒以内に同期されている場合は GNSS UTC 時刻、未同期時は ROS システム時刻へフォールバック
  - `angular_velocity`: rad/s
  - `linear_acceleration`: m/s^2 (G から 9.80665 を乗算して変換)
- `/imu/temperature` (`sensor_msgs/msg/Temperature`): 温度データ
