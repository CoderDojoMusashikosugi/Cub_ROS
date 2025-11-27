# GNSS-based Localization for Nav2

このドキュメントでは、RTK-GNSSとオドメトリを使用したnav2ローカライゼーションシステムの使用方法を説明します。

## 概要

このシステムは、RTK-GNSSの位置情報とオドメトリの角度情報を融合して、nav2で使用可能な`/map -> /base_link`のTF変換を提供します。

### 特徴

- **RTK-GNSS位置の完全信頼**: GNSS状態が有効(STATUS_SBAS_FIX=1またはSTATUS_GBAS_FIX=2)の場合、位置情報を完全に信頼
- **ハイブリッド角度推定**:
  - オドメトリの移動量から角度を積分
  - GNSS移動ベクトルで角度を補正（オプション）
  - RViz 2D Pose Estimateで初期角度を設定可能
- **マップ原点の設定可能性**: 地図の原点座標(緯度・経度)を設定可能
- **シンプルな起動**: 単一のlaunchファイルで全ノードを起動

## システム構成

```
[GNSS受信機]
    ↓ /fix (sensor_msgs/NavSatFix)
[GPS Updater]
    ↓ /gps_pose (geometry_msgs/PoseWithCovarianceStamped)
    ↓
[GNSS-Odom Fusion] ← /odom (nav_msgs/Odometry)
    ↓
[map -> base_link TF]
```

### 起動されるノード

1. **gps_updater_node** (ekf_localizerパッケージ)
   - GNSS座標(緯度・経度)をローカル地図座標系(x, y)に変換
   - 入力: `/fix` (sensor_msgs/NavSatFix)
   - 出力: `/gps_pose` (geometry_msgs/PoseWithCovarianceStamped)

2. **gnss_odom_fusion** (cub_bringupパッケージ)
   - GNSS位置とオドメトリ角度を融合してTF配信
   - 入力: `/gps_pose`, `/odom`, `/fix` (ステータス確認用), `/initialpose` (初期角度設定用)
   - 出力: `map -> base_link` TF, `/gnss_pose` (デバッグ用)
   - 機能: オドメトリ積分 + GNSS移動ベクトル補正

3. **static_transform_publisher**
   - `map -> odom` の恒等変換を配信

## 使用方法

### 1. マップ原点座標の設定

まず、地図の原点となる緯度・経度を設定します。

[cub_navigation/param/gnss_localization.yaml](param/gnss_localization.yaml)を編集:

```yaml
gps_updater:
  ros__parameters:
    use_manual_origin: true

    # ここにマップ原点の座標を設定
    gps_origin_lat: 35.6812      # 緯度(度)
    gps_origin_lon: 139.7671     # 経度(度)
    gps_origin_alt: 40.0         # 高度(m、楕円体高)
```

**重要**: この座標がマップ上の(0, 0)となります。nav2で使用する地図の原点と一致させてください。

### 2. ローカライゼーションの起動

```bash
ros2 launch cub_navigation gnss_as_localization.launch.py
```

このコマンドで、GNSS-オドメトリ融合ローカライゼーションに必要な全てのノードが起動します。

### 3. 初期角度の設定（オプション）

RVizの「2D Pose Estimate」ツールを使用して、ロボットの初期角度を設定できます。

1. RVizで「2D Pose Estimate」ボタンをクリック
2. 地図上でロボットの位置をクリック（位置は無視されます）
3. ドラッグして矢印の向きでロボットの方位を設定
4. 設定した角度が`current_yaw_`に反映されます

**注意**: 位置はGNSSで決定されるため、2D Pose Estimateの位置成分は使用されません。角度のみが反映されます。

### 4. パラメータ調整 (オプション)

必要に応じて、以下のパラメータを調整できます。

#### GNSS Fusion パラメータ

[cub_bringup/params/gnss_fusion.yaml](../cub_bringup/params/gnss_fusion.yaml):

```yaml
gnss_odom_fusion:
  ros__parameters:
    gnss_timeout: 1.0                    # GNSSタイムアウト(秒)
    use_gnss_altitude: false             # 高度をGNSSから取得するか
    tf_publish_rate: 50.0                # TF配信周波数(Hz)
    initial_yaw: 0.0                     # 初期方位角(rad)

    # GNSS移動ベクトルによる角度補正
    use_gnss_yaw_correction: true        # GNSS移動方向で角度を補正
    min_gnss_movement_for_yaw: 2.0       # GNSS移動距離の最小閾値(m)
    min_odom_movement_for_yaw: 2.0       # オドメトリ移動距離の最小閾値(m)
    max_distance_mismatch_ratio: 0.3     # 許容する距離不一致率(0-1)
    gnss_yaw_weight: 0.3                 # 補正の重み(0.0-1.0)
```

#### GPS品質フィルタ パラメータ

[cub_navigation/param/gnss_localization.yaml](param/gnss_localization.yaml):

```yaml
gps_updater:
  ros__parameters:
    max_covariance_threshold: 10.0  # 最大許容誤差(m)
```

## トピック一覧

### 入力トピック

| トピック名 | メッセージ型 | 説明 |
|----------|------------|------|
| `/fix` | sensor_msgs/NavSatFix | GNSS受信機からの位置情報 |
| `/odom` | nav_msgs/Odometry | オドメトリ情報 |
| `/initialpose` | geometry_msgs/PoseWithCovarianceStamped | 初期角度設定（RViz 2D Pose Estimate） |

### 出力トピック

| トピック名 | メッセージ型 | 説明 |
|----------|------------|------|
| `/gps_pose` | geometry_msgs/PoseWithCovarianceStamped | GPS座標変換後の位置 |
| `/gnss_pose` | geometry_msgs/PoseStamped | 融合後の位置(デバッグ用) |

### TF

| 親フレーム | 子フレーム | 説明 |
|----------|----------|------|
| `map` | `base_link` | ロボットの地図上の位置姿勢 |
| `map` | `odom` | 恒等変換(static) |

## GNSS状態の要件

このシステムは、以下の条件を満たすGNSSデータのみを使用します:

1. **ステータス**: `STATUS_SBAS_FIX` (1) または `STATUS_GBAS_FIX` (2)
2. **共分散タイプ**: `COVARIANCE_TYPE_KNOWN` (3) または `COVARIANCE_TYPE_APPROXIMATED` (1)
3. **水平精度**: 標準偏差が`max_covariance_threshold`以下

RTK-GNSSが正常に動作している場合、これらの条件は通常満たされます。

## 角度推定の仕組み

このシステムでは、以下の3つの方法で角度を管理します:

### 1. オドメトリによる積分（基本）

オドメトリの角度変化量を積分することで、ロボットの方位を推定します。

```
delta_yaw = odom_yaw(t) - odom_yaw(t-1)
current_yaw += delta_yaw
```

- **利点**: 高周波数で滑らかな角度更新
- **欠点**: 長時間運用でドリフト蓄積

### 2. GNSS移動ベクトルによる補正（オプション）

ロボットが移動した際、GNSS位置の変化ベクトルから移動方向を計算し、オドメトリ積分値を補正します。

**補正の条件（すべて満たす必要あり）**:
1. GNSS移動距離 ≥ `min_gnss_movement_for_yaw`（前回補正時からの累積）
2. オドメトリ移動距離 ≥ `min_odom_movement_for_yaw`（前回補正時からの累積）
3. 距離不一致率 ≤ `max_distance_mismatch_ratio`

**距離不一致チェック**:
```
mismatch_ratio = |GNSS距離 - Odom距離| / 平均距離
補正実行条件: mismatch_ratio ≤ max_distance_mismatch_ratio (デフォルト: 0.3)
```

この仕組みにより、以下の異常状態で誤補正を防ぎます:
- **ホイールスリップ**: オドメトリ距離 > GNSS距離
- **GNSSグリッチ**: GNSS距離が異常に大きい/小さい
- **低速走行**: 両方の移動距離が閾値未満

**補正式**:
```
gnss_yaw = atan2(Δgnss_y, Δgnss_x)  // 累積移動ベクトルから計算
yaw_correction = (gnss_yaw - current_yaw) * gnss_yaw_weight
current_yaw += yaw_correction
```

- **利点**:
  - オドメトリドリフトの長期的補正
  - 2m以上の移動で高精度（RTK使用時 ±0.6°以下）
  - 異常検知によるロバスト性
- **欠点**:
  - 一定距離以上の移動が必要
  - 低速・停止時は補正なし
- **パラメータ**:
  - `use_gnss_yaw_correction`: 有効/無効（デフォルト: `true`）
  - `min_gnss_movement_for_yaw`: GNSS最小移動距離（デフォルト: `2.0m`）
  - `min_odom_movement_for_yaw`: Odom最小移動距離（デフォルト: `2.0m`）
  - `max_distance_mismatch_ratio`: 最大不一致率（デフォルト: `0.3`）
  - `gnss_yaw_weight`: 補正の重み（デフォルト: `0.3`）

### 3. 2D Pose Estimateによる初期化

RVizの「2D Pose Estimate」から角度を直接設定できます。

- **使用場面**: 起動時、長時間停止後、角度がずれた時
- **動作**: `/initialpose`トピックから角度成分のみを抽出して`current_yaw_`を更新

### 推奨設定

| 使用シーン | `use_gnss_yaw_correction` | `gnss_yaw_weight` | 理由 |
|----------|---------------------------|-------------------|------|
| 直線走行が多い | `true` | `0.3-0.5` | GNSS補正が効果的 |
| カーブが多い | `true` | `0.1-0.2` | 緩やかに補正 |
| 低速・停止が多い | `false` | - | 移動距離不足で補正が働かない |
| 高精度オドメトリ | `false` | - | オドメトリのみで十分 |

## トラブルシューティング

### GNSSデータが使用されない

以下を確認してください:

1. `/fix`トピックにデータが配信されているか
   ```bash
   ros2 topic echo /fix
   ```

2. GNSS statusが1または2になっているか
   ```bash
   ros2 topic echo /fix | grep status
   ```

3. ログでGNSS品質チェックの状態を確認
   ```bash
   ros2 run cub_bringup gnss_odom_fusion --ros-args --log-level debug
   ```

### TFが配信されない

1. オドメトリデータが配信されているか確認
   ```bash
   ros2 topic echo /odom
   ```

2. ノードのログを確認
   ```bash
   ros2 node info /gnss_odom_fusion
   ```

### 位置がおかしい

1. マップ原点座標が正しく設定されているか確認
2. GNSS受信機のアンテナ位置が適切か確認
3. RTK-GNSSのFIX状態を確認

## odom_as_localization.launch.pyとの違い

| 項目 | odom_as_localization | gnss_as_localization |
|-----|---------------------|---------------------|
| 位置情報源 | オドメトリのみ | GNSS + オドメトリ |
| ドリフト | あり | なし(GNSS有効時) |
| 絶対位置精度 | 低 | 高(RTK-GNSS) |
| 使用場所 | 屋内、GNSS不可環境 | 屋外、GNSS利用可能 |

## 関連ファイル

- Launch: [gnss_as_localization.launch.py](launch/gnss_as_localization.launch.py)
- パラメータ: [gnss_localization.yaml](param/gnss_localization.yaml)
- ノード実装: [gnss_odom_fusion.cpp](../cub_bringup/src/gnss_odom_fusion.cpp)
- ヘッダー: [gnss_odom_fusion.hpp](../cub_bringup/include/cub_bringup/gnss_odom_fusion.hpp)
