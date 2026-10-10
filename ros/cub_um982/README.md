# cub_um982

Unicore UM982 デュアルアンテナ GNSS 受信機用 ROS 2 ドライバパッケージ。  
バックエンドとして [`t-nakabayashi/UM982-RTK-GPS-Library`](https://github.com/t-nakabayashi/UM982-RTK-GPS-Library) をサブモジュールとして内包し、NTRIP RTK 補正の送受信とデュアルアンテナによる方位・仰角計測を行います。

## 特徴
- **デュアルアンテナ姿勢出力**: 角度情報（Heading / Pitch）を `geometry_msgs/msg/PoseWithCovarianceStamped` でパブリッシュ（REP-103 ENU 準拠、標準偏差に基づく分散付き）。
- **RTK 測位**: NTRIP クライアントを内包し、単独測位から RTK Float / RTK Fix までのステータスを判別。
- **NavSatFix 出力**: `sensor_msgs/msg/NavSatFix` で経緯度・高度をパブリッシュ。
- **cub_diagnostics 連携**: `/diagnostics` に `UM982 Status` をパブリッシュし、RTK 状態（Fix/Float/Standalone/Timeout）や衛星数、精度指標を監視可能。

---

## トピック

| トピック名 | 型 | 説明 |
|---|---|---|
| `~/pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | デュアルアンテナから算出された姿勢（ENU クォータニオン）および共分散（Pitch/Yaw 分散） |
| `~/fix` | `sensor_msgs/msg/NavSatFix` | 測位座標（WGS84 経緯度・高度）および RTK ステータス |
| `~/fix_secondary` | `sensor_msgs/msg/NavSatFix` | 副アンテナ（ANT2）の測位座標・RTK ステータス（有効な座標を受信した場合のみ） |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | UM982 の動作状態（RTK Fix/Float、衛星数、HDOP、補正データ経過時間など） |

---

## 主なパラメータ (`config/um982.yaml`)

### シリアル / UART 設定
- `port`: シリアルポートパス (デフォルト: `/dev/ttyGPS`)
- `baudrate`: ボーレート (デフォルト: `115200`)
- `serial_timeout`: タイムアウト秒 (デフォルト: `1.0`)
- `output_rate`: 出力レート Hz (デフォルト: `10`)
- `configure_on_start`: 起動時に受信機へ設定コマンドを自動送信するか (デフォルト: `true`)
- `enable_rmc`: RMC センテンス出力を有効にするか (デフォルト: `false`)

### NTRIP / RTK 設定
- `enable_ntrip`: NTRIP 補正を有効にするか (デフォルト: `false`)
- `ntrip_host`: NTRIP キャスターのホスト名または IP
- `ntrip_port`: ポート番号 (デフォルト: `2101`)
- `ntrip_mountpoint`: マウントポイント名
- `ntrip_user`: ユーザー名
- `ntrip_password`: パスワード
- `ntrip_gga_interval`: NTRIP サーバーへの GGA 送信間隔 (デフォルト: `5.0` 秒)

### 座標系・姿勢オフセット
- `frame_id`: メッセージの `frame_id` (デフォルト: `gnss_link`)
- `secondary_frame_id`: 副アンテナの `frame_id` (デフォルト: `gnss_secondary_link`)
- `publish_secondary_fix`: 副アンテナの測位出力を配信するか (デフォルト: `true`)
- `heading_offset_deg`: 車体進行方向に対するアンテナ基線のオフセット角度（度、時計回り正）
- `pitch_offset_deg`: ピッチのオフセット角度（度）

### Diagnostics 設定
- `hardware_id`: ハードウェア ID (デフォルト: `um982`)
- `diagnostic_update_period`: パブリッシュ周期 (デフォルト: `1.0` 秒)
- `diagnostic_timeout`: データ途絶判定タイムアウト (デフォルト: `2.0` 秒)
- `diagnostic_status_name`: ステータス名 (デフォルト: `"UM982 Status"`)

---

## ビルド方法

コンテナ内でビルドを実行します:

```bash
# パッケージ単体ビルド
./docker/run_in_container.sh cbs cub_um982
```

---

## 単体試験・起動方法

### 1. デフォルト設定で起動
```bash
./docker/run_in_container.sh ros2 launch cub_um982 um982.launch.py
```

### 2. 引数でポートや NTRIP を指定して起動
`port` / `baudrate` / `enable_ntrip` / `frame_id` は、引数を指定した場合のみ `config/um982.yaml` の値を上書きします（未指定なら YAML の値を使用）。
```bash
./docker/run_in_container.sh ros2 launch cub_um982 um982.launch.py \
  port:=/dev/ttyUSB0 \
  baudrate:=115200 \
  enable_ntrip:=true
```

### 3. トピックの確認
```bash
# 姿勢と共分散（デュアルアンテナ方位・仰角）
./docker/run_in_container.sh ros2 topic echo /um982_node/pose

# 主アンテナ（ANT1）測位情報
./docker/run_in_container.sh ros2 topic echo /um982_node/fix

# 副アンテナ（ANT2）測位情報（独立出力）
./docker/run_in_container.sh ros2 topic echo /um982_node/fix_secondary

# Diagnostics（受信機ステータス・NTRIP通信統計）
./docker/run_in_container.sh ros2 topic echo /diagnostics
```

屋内などで未測位の場合、`$GNGGAH` を受信していても緯度・経度が空欄になるため、
`fix_secondary` は配信されません。`/diagnostics` の `UM982 Status Secondary Antenna`
で「データを受信したが未測位（WARN）」と「GGAH が届いていない（ERROR）」を確認できます。
衛星数0・測位品質0は未測位です。方位も `SOL_COMPUTED` の解が成立した場合のみ配信します。

起動時には接続中のポートへ `GPGGA 0.1`、`GPGGAH 0.1`、`HEADINGA 0.1` などを送信します
（間隔は `output_rate` に従います）。`configure_on_start: false` の場合は受信機側で出力を設定してください。
`LOG ... ONTIME ...` 形式は実機で構文エラーになるため使用しません。
コマンド仕様は[Unicore N4公式マニュアル](https://en.unicore.com/uploads/file/unicore-reference-commands-manual-for-n4-high-precision-products-v2-en-r1.2.pdf)を参照してください。

---

## cub_diagnostics との連携

`cub_diagnostics/config/example.yaml` などの `diagnostic_aggregator` 設定に以下のように追加することで、Sensors グループに UM982 のステータスが集約されます:

```yaml
diagnostic_aggregator:
  ros__parameters:
    analyzers:
      system:
        type: diagnostic_aggregator/AnalyzerGroup
        path: Robot
        analyzers:
          sensors:
            type: diagnostic_aggregator/GenericAnalyzer
            path: Sensors
            contains: [
              "TopicMonitor",
              "GPS Fix Status",
              "UM982 Status",   # ← これを追加
            ]
```

## UM982の設定

受信機（UM982）内部の不揮発性メモリ（Flash）に設定を書き込み、電源再投入後も意図したポートから必要なデータのみを出力させるための恒久設定手順です。

### 構成と背景

このシステムでは、UM982 の 2 つのポートおよび PPS ピンを用途ごとに分離して使用します。

| ポート / 信号 | 接続先 | 通信速度 | 用途と出力データ |
|---|---|---|---|
| **USB / COM1** | Jetson USB（`/dev/ttyUM982`） | `115200` bps | **ROS 2 測位・姿勢用** (`cub_um982`)<br>・`GPGGA` (10Hz): 主アンテナ測位・RTK状態<br>・`GPGGAH` (10Hz): 副アンテナ測位・RTK状態<br>・`HEADINGA` (10Hz): デュアルアンテナ方位・仰角・標準偏差<br>・`GPSUTCA` (変化時): 閏秒パラメータ<br>・`RECTIMEA` (60s): 受信機UTCオフセット |
| **SERIAL3 / COM3** | Jetson 40pin (`/dev/ttyTHS1`)<br>および Spresense (`Serial2`) | `9600` bps | **システム時刻同期用**<br>・`GPRMC` (1Hz): 日付・時刻情報（NTPsec および TinyGPSPlus 用） |
| **1PPS パルス** | Jetson Pin 32 (`GPIO07`)<br>および Spresense D3 | - | **高精度時刻同期パルス**<br>・1秒周期、立ち上がりエッジ（Positive、パルス幅 100ms） |

主アンテナの `GGA` と副アンテナの `GGAH` はドライバ内で分離します。
`GGAH` を有効にしても主アンテナの測位出力や NTRIP への位置報告には混入しません。
恒久設定では `UNLOG COM1` などで既存出力を停止した後、両アンテナの出力を設定します。

---

### 設定手順

設定方法は **① 付属の設定スクリプト（推奨・一発設定/パイプ対応）** と **② シリアル端末・uPrecise での手動送信** の2通りがあります。

#### 方法 1: 設定スクリプト `configure_um982` による自動設定（推奨）

本パッケージには、安全にディレイを挟みながらコマンドを順次送信し、レスポンスを確認・保存する設定スクリプトが含まれています。

##### ① 推奨デフォルト設定を一括適用する場合（引数なし）
コマンドファイルを用意しなくても、以下の 1 コマンドで下記の推奨設定（基線長 200mm (20cm)、COM1 10Hz、COM3 1Hz GPRMC、1PPS、SAVECONFIG）がすべて適用されます：

```bash
# ROS2 経由で実行（コンテナ内または ROS2 環境）
./docker/run_in_container.sh ros2 run cub_um982 configure_um982 --port /dev/ttyUM982

# またはホスト側から Python スクリプトとして直接実行
./ros/cub_um982/cub_um982/configure_um982.py --port /dev/ttyUSB0
```

##### ② パイプやファイルから任意のコマンド列を流し込む場合
標準入力からのパイプや `--file` オプションにも対応しています：

```bash
# パイプで流し込む場合
cat my_commands.txt | ./docker/run_in_container.sh ros2 run cub_um982 configure_um982 --port /dev/ttyUM982

# ファイルを指定して流し込む場合
./docker/run_in_container.sh ros2 run cub_um982 configure_um982 --port /dev/ttyUM982 --file /home/cub/my_commands.txt

# 送信内容を事前に確認（ドライラン）
./docker/run_in_container.sh ros2 run cub_um982 configure_um982 --dry-run
```

---

#### 方法 2: シリアルツールまたは uPrecise での手動設定

Windows の **uPrecise** ソフトウェア、または Linux のシリアル通信ツール（`minicom`, `cu`, `tio` など）で UM982 のコマンド受付ポート（USB / COM1、初期速度 115200 bps）に接続し、以下のコマンド群を順に送信します。

##### コピペ用一括コマンド列

```text
UNLOG COM1
UNLOG COM2
UNLOG COM3

CONFIG COM1 115200 8 n 1
CONFIG COM3 9600 8 n 1

MODE ROVER
CONFIG HEADING FIXLENGTH
CONFIG HEADING LENGTH 20 2

GPGGA COM1 0.1
GPGGAH COM1 0.1
HEADINGA COM1 0.1
GPSUTCA COM1 ONCHANGED
RECTIMEA COM1 60

GPRMC COM3 1.0

CONFIG PPS ENABLE GPS POSITIVE 100000 1000 0 0

SAVECONFIG
```

---

### 各コマンドの解説

1. **既存設定の全クリア**
   - `UNLOG COM1`, `UNLOG COM2`, `UNLOG COM3`: 各ポートの既存ログ出力を停止します。後続のコマンドで `GPGGA` と `GPGGAH` を有効にします。
2. **シリアルポート通信速度の設定**
   - `CONFIG COM1 115200 8 n 1`: Jetson の `cub_um982` が接続する USB/COM1 を 115200 bps に設定。
   - `CONFIG COM3 9600 8 n 1`: Jetson の NTPsec および Spresense が期待する 9600 bps に設定。
3. **測位・デュアルアンテナ基本動作の設定**
   - `MODE ROVER`: 移動局（Rover）モードに設定。
   - `CONFIG HEADING FIXLENGTH`: アンテナ間距離（基線長）固定モードを有効化。
   - `CONFIG HEADING LENGTH 20 2`: 主アンテナ（ANT1）と副アンテナ（ANT2）の間隔を設定（**基線長 200mm = 20cm、許容誤差 2cm**。UM982 コマンドの単位は cm です）。実機のアンテナ間隔に合わせて値を変更してください。
4. **【COM1 / USB】ROS 2 ドライバ用出力設定**
   - `GPGGA COM1 0.1`: 主アンテナの位置・RTK ステータスを 10Hz（0.1 秒間隔）で出力。
   - `GPGGAH COM1 0.1`: 副アンテナの位置・RTK ステータスを 10Hz で出力。
   - `HEADINGA COM1 0.1`: デュアルアンテナの方位・仰角・標準偏差を 10Hz で出力。
   - `GPSUTCA COM1 ONCHANGED`: GPS-UTC 閏秒パラメータを出力。
   - `RECTIMEA COM1 60`: 受信機内部の UTC 補正オフセットを 60 秒毎に出力。
5. **【COM3 / SERIAL3】Jetson / Spresense 時刻同期用出力設定**
   - `GPRMC COM3 1.0`: 1Hz で NMEA `$GPRMC` センテンスを出力。NTPsec（Jetson）および TinyGPSPlus（Spresense）が年・月・日・時・分・秒を正確に特定するために使用します。
6. **PPS パルス出力設定**
   - `CONFIG PPS ENABLE GPS POSITIVE 100000 1000 0 0`: GPS 基準時刻に同期した 1 秒周期の正極性パルス（パルス幅 100,000 $\mu$s = 100ms）を出力。
7. **設定の永続化**
   - `SAVECONFIG`: 設定を Flash メモリに保存します。次回電源投入時もこの設定で自動起動します。
