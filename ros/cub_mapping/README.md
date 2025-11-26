# cub_mapping

3D点群から2D占有格子地図への変換ツールおよびナビゲーションウェイポイント編集ツールパッケージ

## 概要

このパッケージは、以下の2つの主要機能を提供します：

1. **地図生成ツール**: 3D LiDAR点群データをnav2で使用可能な2D占有格子地図に変換
2. **ウェイポイント編集ツール**: ナビゲーション用のウェイポイントをRViz2上でインタラクティブに配置・編集

高低差のある環境でも、指定した高さ範囲の点群のみを抽出して2Dマップに投影することで、適切な走行可能領域を生成できます。また、生成された地図上でウェイポイントを直感的に配置し、Navigation2で使用可能なYAML形式で保存できます。

### 主な特徴

#### 地図生成ツール
- **インタラクティブな領域選択**: RViz2上でマウスとキーボードを使って直感的に領域を指定
- **リアルタイム可視化**: 選択領域と2D地図をリアルタイムで確認
- **動的な高さ調整**: スクロール操作で2D地図の表示高さを自由に変更可能
- **自動マップサイズ調整**: 点群の範囲に応じて自動的に地図サイズを設定
- **nav2互換**: 生成された地図はnav2でそのまま使用可能

#### ウェイポイント編集ツール
- **視覚的な配置**: RViz2上でマウスクリックによりウェイポイントを配置
- **方向調整**: スクロールホイールでウェイポイントの向き(yaw)を調整
- **プレビュー表示**: 配置前に矢印でウェイポイントの位置と向きを確認
- **複数グループ対応**: ウェイポイントをグループごとに管理
- **2D/3D地図表示**: 2Dマップと3D点群を同時表示して位置合わせ
- **YAMLフォーマット**: cub_behavior_tree互換のYAML形式で保存

## アーキテクチャ

### システム構成

```
┌─────────────────┐
│  PCDファイル    │
└────────┬────────┘
         │
         ▼
┌─────────────────────────────────────┐
│  pcd_to_pointcloud                   │
│  (PCL ROS)                           │
└──────────┬──────────────────────────┘
           │ /cloud_pcd (PointCloud2)
           ▼
┌─────────────────────────────────────┐
│  MapConverterNode                    │
│  (バックエンド)                      │
│  - 点群受信・保存                    │
│  - 領域抽出・2D投影                  │
│  - OccupancyGrid生成                 │
│  - map_slice TFブロードキャスト      │
└──┬──────────┬─────────┬─────────────┘
   │          │         │
   │          │         └─> TF: map → map_slice
   │          │
   │          └─> /occupancy_grid (OccupancyGrid)
   │
   └──> Services: /add_region, /save_map, /clear_map
        Topics: /map_slice_height (購読)

┌─────────────────────────────────────┐
│  RViz2                               │
│  + PointCloudTo2DTool               │
│  (フロントエンド)                    │
│  - インタラクティブUI                │
│  - 高さ情報送信                      │
│  - 領域可視化                        │
└──────────────────────────────────────┘
           │
           └─> /map_slice_height (Float64)
```

### TFフレーム階層

```
map (固定フレーム)
 └─ map_slice (動的フレーム, z軸方向に移動可能)
     └─ OccupancyGrid (2D地図)
```

**重要**: `map_slice`フレームはツールのスクロール操作に応じて高さが変化し、2D地図がその高さで表示されます。これにより、多層階の環境でも各階の地図を確認できます。

## 機能

### 1. RViz2 ツールプラグイン: PointCloudTo2DTool

インタラクティブに点群の領域を選択し、バックエンドノードに送信するツールです。

#### 操作方法

1. **ツールの起動**: RViz2のツールバーから「PointCloudTo2DTool」を選択、または`z`キーを押す

2. **領域の指定**:
   - **マウス移動**: XY平面上の位置を指定
   - **マウススクロール**: 2D地図の高さ（Z方向）を調整
   - **Shift + スクロール**: 領域の幅を調整
   - **Ctrl + スクロール**: 高さ方向の範囲を調整
   - **左クリック**: 現在の設定で領域を確定してバックエンドに送信

3. **可視化**:
   - 黄色の垂直線: カーソル位置（XY）
   - 緑色の半透明ボックス: 選択領域のワイヤフレーム（底面が地図の高さ）

#### ツールプロパティ

RViz2の「Tool Properties」パネルで以下のパラメータを調整できます：

- **Height (Z)**: map_sliceフレームの高さ（m）
  - デフォルト: 0.0
  - 範囲: -10.0 ～ 10.0

- **Width**: XY平面での領域の幅（m）
  - デフォルト: 2.0
  - 範囲: 0.5 ～ 20.0

- **Height Range**: Z方向の抽出範囲（m）
  - デフォルト: 1.0
  - 範囲: 0.1 ～ 5.0
  - 注: 範囲はmap_slice平面から上方向

- **Wireframe Color**: ワイヤフレームの色
  - デフォルト: 緑 (RGB: 0, 255, 0)

- **Auto Send**: マウス移動時に自動で領域を送信
  - デフォルト: false

### 2. バックエンドノード: MapConverterNode

点群データを処理し、2D占有格子地図を生成するノードです。

#### 主要機能

1. **点群の受信と保存**
   - `/cloud_pcd` トピックから点群を受信
   - 全点群をメモリに保持

2. **自動マップサイズ設定**
   - 初回の点群受信時に、点群のバウンディングボックスを計算
   - 10%のマージンを追加してマップサイズを自動設定
   - 原点も点群の中心に合わせて調整

3. **領域抽出と2D投影**
   - 指定された3D領域内の点群を抽出（PCL CropBox使用）
   - Z座標を無視してXY平面に投影
   - 占有格子セルを更新（占有=100, 未知=-1）

4. **TFブロードキャスト**
   - `map` → `map_slice` 変換を10Hzでブロードキャスト
   - ツールから受信した高さ情報に基づいてTFを更新

5. **地図の保存**
   - PGM形式の画像ファイル
   - YAMLメタデータファイル（nav2互換）
   - タイムスタンプ付きディレクトリに保存

## ビルド

```bash
# コンテナ内でビルド
./docker/run_in_container.sh cbs cub_mapping
```

## 使用例

### クイックスタート（Launchファイルを使用）

すべてを一度に起動：

```bash
./docker/run_in_container.sh "source ~/colcon_ws/install/setup.bash && \
  ros2 launch cub_mapping mapping_tool.launch.py"
```

パラメータをカスタマイズ：

```bash
./docker/run_in_container.sh "source ~/colcon_ws/install/setup.bash && \
  ros2 launch cub_mapping mapping_tool.launch.py \
  params_file:=/path/to/custom_params.yaml \
  pcd_file:=/path/to/pointcloud.pcd"
```

### 個別起動

#### 1. バックエンドノードの起動

```bash
./docker/run_in_container.sh "source ~/colcon_ws/install/setup.bash && \
  ros2 run cub_mapping map_converter_node \
  --ros-args --params-file ~/colcon_ws/install/cub_mapping/share/cub_mapping/params/map_converter.yaml"
```

または、パラメータを直接指定：

```bash
./docker/run_in_container.sh "source ~/colcon_ws/install/setup.bash && \
  ros2 run cub_mapping map_converter_node \
  --ros-args -p resolution:=0.05 -p map_frame:=map"
```

#### 2. 点群データの読み込み

```bash
# PCDファイルから点群をパブリッシュ
./docker/run_in_container.sh "source ~/colcon_ws/install/setup.bash && \
  ros2 run pcl_ros pcd_to_pointcloud \
  --ros-args -p file_name:=/path/to/pointcloud.pcd -p tf_frame:=map"
```

#### 3. RViz2の起動

```bash
./docker/run_in_container.sh "source ~/colcon_ws/install/setup.bash && \
  ros2 run rviz2 rviz2 \
  -d ~/colcon_ws/install/cub_mapping/share/cub_mapping/rviz/map_converter.rviz"
```

#### 4. RViz2でツールを使用

1. RViz2のツールバーから「PointCloudTo2DTool」を選択、または`z`キーを押す
2. マウスとキーボードで領域を指定：
   - マウス移動: XY位置
   - スクロール: 地図の高さ
   - Shift+スクロール: 幅
   - Ctrl+スクロール: 高さ範囲
3. 左クリックで領域を追加
4. 必要に応じて`/save_map`サービスで地図を保存

## パラメータ設定

### map_converter.yaml

```yaml
map_converter_node:
  ros__parameters:
    # 地図の解像度（m/セル）
    resolution: 0.2

    # 地図サイズ（m）※点群受信後に自動設定されるため通常は変更不要
    map_width: 50.0
    map_height: 50.0

    # 地図原点（m）※点群受信後に自動設定されるため通常は変更不要
    origin_x: -25.0
    origin_y: -25.0

    # 固定フレームID
    map_frame: "map"

    # 購読する点群トピック
    pointcloud_topic: "/cloud_pcd"

    # 地図保存先ディレクトリ
    output_directory: "/home/cub/maps"

    # OccupancyGridのパブリッシュ周期（Hz）
    publish_rate: 1.0

    # map_sliceフレームの初期高さ（m）
    slice_height: 0.0

    # TFブロードキャスト周期（Hz）
    tf_publish_rate: 10.0
```

## サービスインターフェース

### AddRegion.srv

領域追加リクエスト:
```
float64 center_x        # 領域中心のX座標（m）
float64 center_y        # 領域中心のY座標（m）
float64 center_z        # 領域中心の高さ（m）
float64 width           # XY平面での幅（m）
float64 height_range    # Z方向の範囲（m）
---
bool success            # 処理成功フラグ
string message          # ステータスメッセージ
int32 points_processed  # 処理した点の数
```

## 提供サービス

### map_converter_node

- **`/add_region`** (`cub_mapping/srv/AddRegion`)
  - 指定された領域の点群を2Dマップに追加
  - RViz2ツールから自動的に呼び出されます

- **`/save_map`** (`std_srvs/srv/Trigger`)
  - 現在の地図をPGM+YAML形式で保存
  - タイムスタンプ付きディレクトリに保存

  ```bash
  ./docker/run_in_container.sh "source ~/colcon_ws/install/setup.bash && \
    ros2 service call /save_map std_srvs/srv/Trigger"
  ```

- **`/clear_map`** (`std_srvs/srv/Trigger`)
  - 地図をクリア（すべてのセルを未知状態に戻す）

  ```bash
  ./docker/run_in_container.sh "source ~/colcon_ws/install/setup.bash && \
    ros2 service call /clear_map std_srvs/srv/Trigger"
  ```

## パブリッシュされるトピック

### map_converter_node

- **`/occupancy_grid`** (`nav_msgs/msg/OccupancyGrid`)
  - 生成された2D占有格子地図（nav2互換）
  - フレームID: `map_slice`
  - パブリッシュ周期: 1.0 Hz（デフォルト）

- **`/region_markers`** (`visualization_msgs/msg/Marker`)
  - RViz2で処理済み領域を可視化するマーカー
  - フレームID: `map`

## 購読されるトピック

### map_converter_node

- **`/cloud_pcd`** (`sensor_msgs/msg/PointCloud2`)
  - 入力点群データ
  - 初回受信時に地図サイズを自動設定

- **`/map_slice_height`** (`std_msgs/msg/Float64`)
  - map_sliceフレームの高さ情報
  - PointCloudTo2DTool から送信される

## TF (Transform Frames)

### ブロードキャスト

- **`map` → `map_slice`**
  - 親フレーム: `map`
  - 子フレーム: `map_slice`
  - ブロードキャスト周期: 10.0 Hz
  - Z軸方向のオフセット: `slice_height` パラメータ値
  - 用途: 2D地図の表示高さを動的に変更

---

# ウェイポイント編集ツール

## 概要

Waypoint Editor Toolは、Navigation2で使用するウェイポイント（経由点）をRViz2上でインタラクティブに作成・編集するためのツールです。2Dマップと3D点群を参照しながら、直感的にウェイポイントを配置できます。

## システム構成

```
┌─────────────────┐         ┌─────────────────┐
│  2D Map (YAML)  │         │  3D PCD File    │
└────────┬────────┘         └────────┬────────┘
         │                           │
         ▼                           ▼
┌─────────────────┐         ┌─────────────────┐
│  map_server     │         │ pcd_to_         │
│  (Nav2)         │         │ pointcloud      │
└────────┬────────┘         └────────┬────────┘
         │                           │
         │ /map                      │ /cloud_pcd_waypoint
         │                           │
         └───────────┬───────────────┘
                     │
                     ▼
         ┌───────────────────────────┐
         │  RViz2                     │
         │  + WaypointEditTool        │
         │  (フロントエンド)          │
         │  - プレビュー矢印          │
         │  - マウス/キーボード操作   │
         └───────────┬───────────────┘
                     │ Services
                     ▼
         ┌───────────────────────────┐
         │  waypoint_manager_node    │
         │  (バックエンド)            │
         │  - ウェイポイント管理      │
         │  - YAML I/O               │
         │  - マーカー可視化          │
         └───────────────────────────┘
```

## クイックスタート

### 1. ツールの起動

```bash
# 既存のウェイポイントファイルを読み込んで起動
./docker/run_in_container.sh "source ~/colcon_ws/install/setup.bash && \
  ros2 launch cub_mapping waypoint_editor.launch.py"
```

デフォルトでは以下のファイルが読み込まれます:
- ウェイポイント: `/home/cub/colcon_ws/src/cub/cub_behavior_tree/routes/3d_waypoints.yaml`
- 2Dマップ: `/home/cub/maps/20251126_123706/map.yaml`
- 3D点群: `/home/cub/maps/map_tc25_gnss_0_3.pcd`

カスタムファイルを指定する場合:

```bash
./docker/run_in_container.sh "source ~/colcon_ws/install/setup.bash && \
  ros2 launch cub_mapping waypoint_editor.launch.py \
  waypoint_file:=/path/to/waypoints.yaml \
  map_file:=/path/to/map.yaml \
  pcd_file:=/path/to/pointcloud.pcd"
```

### 2. RViz2でウェイポイントを配置

#### 基本操作

1. **ツールを有効化**: RViz2のツールバーから「WaypointEditTool」を選択、または `w` キーを押す

2. **ウェイポイントを配置**:
   - **マウス移動**: カーソル位置にプレビュー矢印が表示されます
   - **マウススクロール**: ウェイポイントの向き(yaw角)を調整
   - **左クリック**: 現在の位置と向きでウェイポイントを配置

3. **配置されたウェイポイント**: RViz2上に矢印マーカーとして表示されます

#### キーボードショートカット

| キー | 機能 |
|------|------|
| `w` | ツールを有効化 |
| `a` | 追加モード (Add Waypoint) |
| `s` | 選択モード (Select) ※将来実装予定 |
| `m` | 移動モード (Move Position) ※将来実装予定 |
| `r` | 回転モード (Rotate Orientation) ※将来実装予定 |

#### Tool Properties パネル

RViz2の「Tool Properties」パネルで以下を調整できます:

- **Edit Mode**: 編集モード選択（現在は Add Waypoint のみ）
- **Current Group**: 追加先のウェイポイントグループ
- **Default Yaw**: デフォルトの向き角度（-π ～ π rad）
- **Preview Color**: プレビュー矢印の色（デフォルト: シアン）
- **Arrow Scale**: 矢印のサイズ（0.1 ～ 2.0）

### 3. ウェイポイントの保存

ウェイポイントは自動保存されません。サービスを呼び出して手動で保存してください:

```bash
./docker/run_in_container.sh "source ~/colcon_ws/install/setup.bash && \
  ros2 service call /save_waypoints std_srvs/srv/Trigger"
```

## ウェイポイント形式

### YAML フォーマット

cub_behavior_tree互換のYAML形式を使用します:

```yaml
waypoint_groups:
  - group_name: "Start Path"
    require_input: false
    waypoints:
      - x: 36.507
        y: 3.542
        yaw: 1.530
      - x: 37.123
        y: 4.678
        yaw: 0.785

  - group_name: "Return Path"
    require_input: true
    waypoints:
      - x: 38.456
        y: 5.234
        yaw: -1.570
```

### フィールド説明

- `waypoint_groups`: ウェイポイントグループのリスト
  - `group_name`: グループの名前（文字列）
  - `require_input`: ユーザー入力が必要かどうか（真偽値）
  - `waypoints`: ウェイポイントのリスト
    - `x`: X座標（m、mapフレーム基準）
    - `y`: Y座標（m、mapフレーム基準）
    - `yaw`: 向き角度（rad、-π ～ π）

## サービスインターフェース

### waypoint_manager_node 提供サービス

#### ウェイポイント操作

- **`/add_waypoint`** (`cub_mapping/srv/AddWaypoint`)
  ```
  int32 group_index
  float64 x
  float64 y
  float64 yaw
  ---
  bool success
  string message
  int32 waypoint_index
  ```
  指定グループにウェイポイントを追加

- **`/delete_waypoint`** (`cub_mapping/srv/DeleteWaypoint`)
  ```
  int32 group_index
  int32 waypoint_index
  ---
  bool success
  string message
  ```
  ウェイポイントを削除

- **`/move_waypoint`** (`cub_mapping/srv/MoveWaypoint`)
  ```
  int32 group_index
  int32 waypoint_index
  float64 new_x
  float64 new_y
  ---
  bool success
  string message
  ```
  ウェイポイントの位置を変更

- **`/update_waypoint_yaw`** (`cub_mapping/srv/UpdateWaypointYaw`)
  ```
  int32 group_index
  int32 waypoint_index
  float64 new_yaw
  ---
  bool success
  string message
  ```
  ウェイポイントの向きを変更

- **`/insert_waypoint`** (`cub_mapping/srv/InsertWaypoint`)
  ```
  int32 group_index
  int32 insert_index
  float64 x
  float64 y
  float64 yaw
  ---
  bool success
  string message
  ```
  指定位置にウェイポイントを挿入

#### グループ操作

- **`/create_group`** (`cub_mapping/srv/CreateGroup`)
  ```
  string group_name
  bool require_input
  ---
  bool success
  string message
  int32 group_index
  ```
  新しいグループを作成

- **`/delete_group`** (`cub_mapping/srv/DeleteGroup`)
  ```
  int32 group_index
  ---
  bool success
  string message
  ```
  グループを削除

- **`/rename_group`** (`cub_mapping/srv/RenameGroup`)
  ```
  int32 group_index
  string new_name
  ---
  bool success
  string message
  ```
  グループ名を変更

- **`/set_require_input`** (`cub_mapping/srv/SetRequireInput`)
  ```
  int32 group_index
  bool require_input
  ---
  bool success
  string message
  ```
  グループのrequire_inputフラグを設定

#### ファイル操作

- **`/load_waypoints`** (`cub_mapping/srv/LoadWaypoints`)
  ```
  string filepath
  ---
  bool success
  string message
  int32 num_groups
  ```
  YAMLファイルからウェイポイントを読み込み

## CLI操作例

### グループの作成

```bash
./docker/run_in_container.sh "source ~/colcon_ws/install/setup.bash && \
  ros2 service call /create_group cub_mapping/srv/CreateGroup \
  '{group_name: \"Test Path\", require_input: false}'"
```

### ウェイポイントの追加

```bash
./docker/run_in_container.sh "source ~/colcon_ws/install/setup.bash && \
  ros2 service call /add_waypoint cub_mapping/srv/AddWaypoint \
  '{group_index: 0, x: 10.0, y: 20.0, yaw: 1.57}'"
```

### ウェイポイントの削除

```bash
./docker/run_in_container.sh "source ~/colcon_ws/install/setup.bash && \
  ros2 service call /delete_waypoint cub_mapping/srv/DeleteWaypoint \
  '{group_index: 0, waypoint_index: 2}'"
```

## パラメータ設定

### waypoint_editor.yaml

```yaml
waypoint_manager_node:
  ros__parameters:
    # 可視化のフレームID
    frame_id: "map"

    # 自動ロードするウェイポイントファイル（空 = ロードしない）
    waypoint_file: ""

    # マーカー配信周期（Hz）
    publish_rate: 10.0

    # 変更時に自動保存
    auto_save: false

    # 可視化設定
    show_waypoint_numbers: true  # ウェイポイント番号を表示
    show_connections: true       # ウェイポイント間の線を表示
```

## パブリッシュされるトピック

- **`/waypoint_markers`** (`visualization_msgs/msg/MarkerArray`)
  - ウェイポイントの可視化マーカー
  - グループごとに異なる色（虹色）で表示
  - 矢印マーカー（位置と向き）
  - テキストマーカー（ウェイポイント番号）
  - ラインマーカー（ウェイポイント間の接続線）

## トラブルシューティング

### ウェイポイントが配置できない

1. **ツールが有効化されているか確認**:
   - RViz2のツールバーで「WaypointEditTool」が選択されているか確認
   - または `w` キーを押してツールを有効化

2. **サービスが利用可能か確認**:
   ```bash
   ros2 service list | grep add_waypoint
   ```

3. **バックエンドノードが起動しているか確認**:
   ```bash
   ros2 node list | grep waypoint_manager
   ```

### マーカーが表示されない

1. **トピックが配信されているか確認**:
   ```bash
   ros2 topic echo /waypoint_markers --once
   ```

2. **RViz2でトピックが購読されているか確認**:
   - Displaysパネルで「Waypoint Markers」が有効になっているか
   - トピック名が `/waypoint_markers` になっているか

### プレビュー矢印が表示されない

1. **ツールが有効化されているか確認**
2. **カメラがmap平面を見ているか確認**: 地面（Z=0）との交差判定が必要

## 将来の機能拡張

Phase 3以降で以下の機能を実装予定:

- [ ] **Select Mode**: クリックでウェイポイントを選択
- [ ] **Move Position Mode**: ドラッグでウェイポイントを移動
- [ ] **Rotate Orientation Mode**: ドラッグまたはスクロールで向きを変更
- [ ] **Undo/Redo**: 操作の取り消し・やり直し
- [ ] **Copy/Paste**: ウェイポイントのコピー＆ペースト
- [ ] **Multi-selection**: 複数ウェイポイントの同時選択・移動

---

# 地図生成ツール（既存機能）

## ディレクトリ構成

```
cub_mapping/
├── CMakeLists.txt                     # ビルド設定
├── package.xml                        # パッケージメタデータ
├── README.md                          # このファイル
├── include/cub_mapping/
│   ├── pointcloud_to_2d_tool.hpp      # 地図生成ツールヘッダー
│   ├── map_converter_node.hpp         # 地図生成バックエンドヘッダー
│   ├── waypoint_edit_tool.hpp         # ウェイポイント編集ツールヘッダー
│   ├── waypoint_manager_node.hpp      # ウェイポイント管理バックエンドヘッダー
│   └── waypoint_types.hpp             # ウェイポイントデータ構造定義
├── src/
│   ├── pointcloud_to_2d_tool.cpp      # 地図生成ツール実装
│   ├── map_converter_node.cpp         # 地図生成バックエンド実装
│   ├── waypoint_edit_tool.cpp         # ウェイポイント編集ツール実装
│   └── waypoint_manager_node.cpp      # ウェイポイント管理バックエンド実装
├── srv/
│   ├── AddRegion.srv                  # 地図生成用サービス
│   ├── AddWaypoint.srv                # ウェイポイント追加
│   ├── DeleteWaypoint.srv             # ウェイポイント削除
│   ├── MoveWaypoint.srv               # ウェイポイント移動
│   ├── UpdateWaypointYaw.srv          # ウェイポイント方向更新
│   ├── InsertWaypoint.srv             # ウェイポイント挿入
│   ├── CreateGroup.srv                # グループ作成
│   ├── DeleteGroup.srv                # グループ削除
│   ├── RenameGroup.srv                # グループ名変更
│   ├── SetRequireInput.srv            # require_inputフラグ設定
│   └── LoadWaypoints.srv              # ウェイポイント読み込み
├── launch/
│   ├── mapping_tool.launch.py         # 地図生成ツール起動
│   └── waypoint_editor.launch.py      # ウェイポイント編集ツール起動
├── params/
│   ├── map_converter.yaml             # 地図生成パラメータ
│   └── waypoint_editor.yaml           # ウェイポイント編集パラメータ
├── rviz/
│   ├── map_converter.rviz             # 地図生成用RViz設定
│   └── waypoint_editor.rviz           # ウェイポイント編集用RViz設定
└── plugins_description.xml            # RVizプラグイン登録
```

## 実装詳細

### バックエンド主導のTF管理

このパッケージでは、TF（Transform Frame）の管理をバックエンドノード（MapConverterNode）に集約する設計を採用しています。

**メリット:**
- ツール非起動時でも2D地図が表示される
- 責任の明確な分離（バックエンド=TF管理、フロントエンド=UI操作）
- ツール再起動時も地図の高さが維持される

**通信フロー:**
1. ユーザーがRViz2ツールでスクロール操作
2. ツールが`/map_slice_height`トピックに高さ情報をパブリッシュ
3. バックエンドノードが高さ情報を受信して`map_slice` TFを更新
4. RViz2が更新されたTFに基づいて2D地図を描画

### 領域抽出アルゴリズム

1. **領域定義**: ユーザーが指定した中心座標(x, y, z)、幅(w)、高さ範囲(h)
2. **CropBox適用**: PCLのCropBoxフィルタで3D領域内の点群を抽出
   - X範囲: [x - w/2, x + w/2]
   - Y範囲: [y - w/2, y + w/2]
   - Z範囲: [z - h/2, z + h/2]
3. **2D投影**: 抽出された点のZ座標を無視してXY平面に投影
4. **グリッド更新**: 対応する占有格子セルを「占有」(100)に設定

### 地図の保存形式

保存される地図は以下の構成：

```
/home/cub/maps/YYYYMMDD_HHMMSS/
├── map.pgm      # 画像ファイル（8bit グレースケール）
└── map.yaml     # メタデータファイル
```

**map.yaml の内容:**
```yaml
image: map.pgm
resolution: 0.2           # m/セル
origin: [-25.0, -25.0, 0.0]  # [x, y, θ]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```

## トラブルシューティング

### 地図が表示されない

1. **点群が受信されているか確認:**
   ```bash
   ros2 topic echo /cloud_pcd --once
   ```

2. **TFフレームが存在するか確認:**
   ```bash
   ros2 run tf2_ros tf2_echo map map_slice
   ```

3. **OccupancyGridがパブリッシュされているか確認:**
   ```bash
   ros2 topic echo /occupancy_grid --once
   ```

### ツールが反応しない

1. **ツールが起動されているか確認** (RViz2のツールバーで確認)
2. **サービスが利用可能か確認:**
   ```bash
   ros2 service list | grep add_region
   ```

### 地図が空のまま

1. **領域を追加しているか確認** (左クリックで領域を追加)
2. **指定した高さに点群が存在するか確認** (RViz2で3D点群を確認)

## 依存関係

- ROS2 Humble
- RViz2
- PCL (Point Cloud Library) 1.12+
- PCL ROS 2.4+
- Ogre3D (RViz2の依存)
- Qt5
- OpenCV 4.5+
- TF2

## ライセンス

Apache-2.0
