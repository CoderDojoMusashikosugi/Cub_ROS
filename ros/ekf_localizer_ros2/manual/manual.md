<artifact identifier="ekf-manual-md" type="application/vnd.ant.code" language="markdown" title="EKF自己位置推定システム操作マニュアル">
# EKF自己位置推定システム 操作マニュアル

## 目次
1. [システム概要](#1-システム概要)
2. [各ノードの役割](#2-各ノードの役割)
3. [事前準備](#3-事前準備)
4. [パラメータ調整ガイド](#4-パラメータ調整ガイド)
5. [トラブルシューティング](#5-トラブルシューティング)

---

## 1. システム概要

このシステムは、拡張カルマンフィルタ(EKF)を用いて自律移動ロボットの自己位置を推定します。

**使用センサ:**
- LiDAR（点群データ）
- IMU（角速度）
- オドメトリ（車輪速度）
- GPS（オプション）

---

## 2. 各ノードの役割

### **ekf_localizer_node**
- **役割**: センサ情報を統合して最終的な位置推定を行う中核ノード
- **入力**: IMU、オドメトリ、NDT位置、GPS位置
- **出力**: 推定位置 (`/test/ekf_pose`)
- **処理内容**:
  - 動作更新: IMU/オドメトリから位置を予測
  - 観測更新: NDT/GPSの測定値で位置を補正

### **map_matcher_node**
- **役割**: LiDAR点群と地図をマッチングして位置を推定
- **入力**: LiDAR点群、EKF位置、点群地図(PCD)
- **出力**: NDT推定位置 (`/test/ndt_pose`)
- **処理内容**:
  - NDT(Normal Distributions Transform)アルゴリズムで点群マッチング

### **gps_updater_node**
- **役割**: GPS座標をローカル座標系に変換
- **入力**: GPS信号 (`/fix`)
- **出力**: ローカル座標のGPS位置 (`/gps_pose`)
- **処理内容**:
  - GPS品質チェック（衛星数、測位精度）
  - 緯度経度をメートル単位の座標に変換

### **tf_cub_node**
- **役割**: 座標変換の配信
- **処理内容**: `map` → `base_link` のTF変換を配信

---

## 3. 事前準備

### 3.1 点群地図の準備

1. **地図ファイルの配置**
   ```bash
   # PCDファイルを指定のパスに配置
   /home/cub/colcon_ws/src/cub/ekf_localizer_ros2/pcd/tsukuba-map.pcd
   ```

2. **地図パスの設定**
   ```yaml
   # map_matcher.yaml
   pcd_file_path: /your/path/to/map.pcd
   ```

### 3.2 初期位置の設定

```yaml
# ekf.yaml
INIT_X: 0.0      # 初期X座標 [m]
INIT_Y: 0.0      # 初期Y座標 [m]
INIT_YAW: 0.0    # 初期方位 [rad]
```

### 3.3 GPS原点の設定（GPS使用時）

```yaml
# gps_updater.yaml
use_manual_origin: false  # 自動設定の場合
# use_manual_origin: true   # 手動設定の場合
# gps_origin_lat: 35.6812   # 緯度
# gps_origin_lon: 139.7671  # 経度
```

### 3.4 起動

```bash
ros2 launch ekf_localizer ekf_locali.launch.py
```

---

## 4. パラメータ調整ガイド

### 4.1 計算負荷の調整

#### **軽量化したい場合**

| パラメータ | ファイル | 推奨値 | 効果 |
|----------|---------|--------|------|
| `VOXEL_SIZE` | map_matcher.yaml | 0.4～0.5 | 大きくすると処理点数が減る |
| `VOXEL_SIZE_MAP` | map_matcher.yaml | 0.5～0.6 | 地図の点数を削減 |
| `LIMIT_RANGE` | map_matcher.yaml | 30.0 | 処理範囲を狭める |
| `MAX_ITERATION` | map_matcher.yaml | 20 | 最適化の繰り返し回数を減らす |
| `RESOLUTION` | map_matcher.yaml | 2.0 | NDTのボクセルサイズを大きく |

#### **精度重視（負荷増）の場合**

| パラメータ | ファイル | 推奨値 | 効果 |
|----------|---------|--------|------|
| `VOXEL_SIZE` | map_matcher.yaml | 0.1～0.2 | 細かい点群を保持 |
| `VOXEL_SIZE_MAP` | map_matcher.yaml | 0.2～0.3 | 詳細な地図を保持 |
| `LIMIT_RANGE` | map_matcher.yaml | 50.0 | 広範囲をマッチング |
| `MAX_ITERATION` | map_matcher.yaml | 40～50 | 最適化を徹底 |

**負荷の目安:**
- CPU使用率が高い → `VOXEL_SIZE`と`VOXEL_SIZE_MAP`を大きく
- リアルタイム性が低い → `MAX_ITERATION`を減らす

---

### 4.2 位置推定精度の調整

#### **推定が不安定な場合**

**症状1: 位置がジャンプする**

```yaml
# ekf.yaml
TH_MAHALANOBIS: 3.0  # デフォルト1.5から増やす
```
→ 外れ値の測定を受け入れやすくする

**症状2: 地図とずれる**

```yaml
# map_matcher.yaml
RESOLUTION: 1.0～1.5  # デフォルトから調整
STEP_SIZE: 0.05       # 小さくして慎重に最適化
```

**症状3: GPS測位が不安定**

```yaml
# gps_updater.yaml
max_covariance_threshold: 15.0  # デフォルト10.0から緩和
```

```yaml
# ekf.yaml
SIGMA_GPS: 5.0  # デフォルト3.0から増やす（GPS信頼度を下げる）
```

#### **推定の応答性を調整**

**センサノイズパラメータ（小さいほど信頼）:**

```yaml
# ekf.yaml
SIGMA_IMU: 0.0001   # IMUの信頼度
SIGMA_ODOM: 0.0001  # オドメトリの信頼度
SIGMA_NDT: 0.0001   # NDTの信頼度
SIGMA_GPS: 3.0      # GPSの信頼度
```

- 値を**小さく** → そのセンサを信頼して素早く反映
- 値を**大きく** → ノイズとして扱い緩やかに反映

**動作ノイズパラメータ:**

```yaml
# ekf.yaml
MOTION_NOISE_NN: 0.01   # 並進のノイズ
MOTION_NOISE_OO: 0.0001 # 回転のノイズ
```

---

### 4.3 NDTスキャンマッチング精度が不安定な時の対応

NDTマッチングの精度が不安定な場合、以下の診断と対策を順番に試してください。

#### **診断方法**

ログで以下を確認:
```bash
# FitnessScoreを確認（小さいほど良い）
# Has converged（収束したか）
# マッチング後の位置のジャンプ
```

---

#### **対策1: NDTパラメータの基本調整**

**症状: マッチングが収束しない（Has converged が出る）**

```yaml
# map_matcher.yaml
MAX_ITERATION: 50      # 繰り返し回数を増やす（デフォルト30）
TRANS_EPSILON: 0.0001  # 収束判定を厳しく（デフォルト0.001）
```

**症状: マッチング結果が不正確（FitnessScoreが高い）**

```yaml
# map_matcher.yaml
RESOLUTION: 1.0～1.5   # NDTボクセルサイズを調整
STEP_SIZE: 0.05        # 最適化ステップを小さく（慎重に探索）
```

---

#### **対策2: 点群データの改善**

**入力点群の品質向上:**

```yaml
# map_matcher.yaml
VOXEL_SIZE: 0.2～0.3   # ダウンサンプリングを控えめに
                        # 小さいほど詳細な点群を保持
```

**地図点群の品質確認:**

```yaml
# map_matcher.yaml
VOXEL_SIZE_MAP: 0.3～0.4  # 地図の解像度を上げる
```

**処理範囲の最適化:**

```yaml
# map_matcher.yaml
LIMIT_RANGE: 40.0～60.0   # 広範囲でマッチング
                           # 特徴の少ない環境では広めに設定
```

---

#### **対策3: 初期推定値の改善**

NDTは初期推定値に依存するため、EKFの推定精度も重要です。

**オドメトリの信頼度を上げる:**

```yaml
# ekf.yaml
SIGMA_ODOM: 0.00005   # オドメトリを信頼（デフォルト0.0001）
```

**マハラノビス距離の閾値を調整:**

```yaml
# ekf.yaml
TH_MAHALANOBIS: 2.0～2.5  # NDT結果を受け入れやすく
```

---

#### **対策4: 環境別のチューニング**

**特徴の少ない環境（廊下、平坦な場所）:**

```yaml
# map_matcher.yaml
RESOLUTION: 2.0～3.0      # 大きめのボクセルで大局的に
LIMIT_RANGE: 60.0         # 広範囲でマッチング
VOXEL_SIZE: 0.15          # 入力点群は細かく保持
MAX_ITERATION: 50         # 繰り返し回数を増やす
```

**特徴の多い環境（構造物が多い）:**

```yaml
# map_matcher.yaml
RESOLUTION: 1.0～1.5      # 細かいボクセルで精密に
LIMIT_RANGE: 40.0         # 適度な範囲
VOXEL_SIZE: 0.25          # 計算負荷とのバランス
MAX_ITERATION: 30         # 通常の繰り返し回数
```

**動的障害物が多い環境:**

```yaml
# ekf.yaml
SIGMA_NDT: 0.001          # NDT信頼度を下げる（デフォルト0.0001）
TH_MAHALANOBIS: 2.5       # 外れ値を棄却しやすく
```

---

#### **対策5: 地図品質の確認**

NDT精度は地図品質に大きく依存します。

**地図の問題をチェック:**
1. [ ] 地図が古くないか（環境変化）
2. [ ] 地図の密度が適切か（疎すぎ/密すぎ）
3. [ ] 地図に動的物体が含まれていないか
4. [ ] 地図の座標系オフセットが正しいか

**地図のダウンサンプリング確認:**

```yaml
# map_matcher.yaml
VOXEL_SIZE_MAP: 0.3   # 小さくして地図を詳細に
                       # ただし計算負荷とトレードオフ
```

**地図オフセットの調整:**

```yaml
# map_matcher.yaml
MAP_OFFSET_X: 0.0
MAP_OFFSET_Y: 0.0
MAP_OFFSET_YAW: 0.0   # 地図の回転オフセット（rad）
```

---

#### **対策6: EKFとNDTの協調調整**

**NDT結果の採用条件を調整:**

```yaml
# ekf.yaml
# 共分散が大きい時はNDT結果を強制採用
TH_POSE_COVARIANCE: 0.4      # 位置分散の閾値
TH_DIRECTION_COVARIANCE: 0.2  # 方位分散の閾値
```

**NDT結果の重み付け:**

```yaml
# ekf.yaml
SIGMA_NDT: 0.0001   # 小さい→NDTを強く信頼
                     # 大きい→NDTの影響を抑える
```

---

#### **診断フローチャート**

```
NDT精度が不安定
    ↓
[1] FitnessScoreを確認
    ├→ 高い（>1.0）: RESOLUTION、STEP_SIZEを調整
    └→ 低いが不安定: VOXEL_SIZE、LIMIT_RANGEを調整
    ↓
[2] 収束状況を確認
    ├→ 収束しない: MAX_ITERATION、TRANS_EPSILONを調整
    └→ 収束するが誤差大: 初期推定値（EKF）を改善
    ↓
[3] 環境を確認
    ├→ 特徴少ない: 広範囲マッチング、大きいRESOLUTION
    ├→ 動的障害物多い: SIGMA_NDT増、TH_MAHALANOBIS増
    └→ 地図古い: 地図を再作成
```

---

#### **推奨されるNDT精度チューニング手順**

1. **まず基本設定で動作確認**
   ```yaml
   RESOLUTION: 1.5
   VOXEL_SIZE: 0.3
   LIMIT_RANGE: 40.0
   MAX_ITERATION: 30
   ```

2. **FitnessScoreをログで監視**
   - 0.5未満: 良好
   - 0.5～1.0: やや不安定
   - 1.0以上: 要調整

3. **環境に応じてRESOLUTIONを調整**
   - 特徴少ない → 大きく（2.0～3.0）
   - 特徴多い → 小さく（1.0～1.5）

4. **計算負荷が許せばVOXEL_SIZEを小さく**
   - 0.2～0.3で高精度化

5. **収束しない場合はMAX_ITERATIONを増やす**
   - 40～50まで増やしてテスト

---

### 4.4 環境別の推奨設定

#### **屋内環境**
- GPS無効: `GPS_MEASUREMENT_ENABLE: false`
- NDT精度重視: `RESOLUTION: 1.0`, `VOXEL_SIZE: 0.2`

#### **屋外環境**
- GPS有効: `GPS_MEASUREMENT_ENABLE: true`
- 広範囲マッチング: `LIMIT_RANGE: 50.0`

#### **高速移動**
- 応答性重視: `SIGMA_ODOM: 0.00005`（オドメトリを信頼）
- マハラノビス距離緩和: `TH_MAHALANOBIS: 2.0`

---

## 5. トラブルシューティング

### Q1. 位置推定が始まらない

**確認事項:**
- [ ] 点群地図が読み込めているか
- [ ] LiDAR/IMU/オドメトリのトピックが配信されているか
- [ ] 初期位置が地図内にあるか

**ログ確認:**
```bash
ros2 topic echo /test/ekf_pose
ros2 topic echo /test/ndt_pose
```

### Q2. NDTマッチングが失敗する

**原因と対策:**
- 点群が少ない → `LIMIT_RANGE`を広げる
- 地図とずれている → 初期位置を再設定
- 収束しない → `MAX_ITERATION`を増やす、`STEP_SIZE`を小さく
- FitnessScoreが高い → `RESOLUTION`を調整、地図品質を確認

**詳細は [4.3 NDTスキャンマッチング精度が不安定な時の対応](#43-ndtスキャンマッチング精度が不安定な時の対応) を参照**

### Q3. GPS測位が採用されない

**ログで確認:**
```
GPS quality check failed
GPS: Horizontal accuracy too low
```

**対策:**
```yaml
# gps_updater.yaml
max_covariance_threshold: 15.0  # 緩和
```

### Q4. 計算が重すぎる

**即効性のある対策:**
1. `VOXEL_SIZE: 0.5`
2. `VOXEL_SIZE_MAP: 0.6`
3. `MAX_ITERATION: 20`
4. `LIMIT_RANGE: 30.0`

---

## 付録: 主要パラメータ一覧表

### EKFパラメータ

| パラメータ | 影響 | 小さい値 | 大きい値 |
|-----------|------|---------|---------|
| `SIGMA_IMU` | IMU信頼度 | 高信頼/応答速い | 低信頼/応答遅い |
| `SIGMA_ODOM` | オドメトリ信頼度 | 高信頼/応答速い | 低信頼/応答遅い |
| `SIGMA_NDT` | NDT信頼度 | 高信頼/応答速い | 低信頼/応答遅い |
| `SIGMA_GPS` | GPS信頼度 | 高信頼/応答速い | 低信頼/応答遅い |
| `TH_MAHALANOBIS` | 安定性 | 厳格/不安定 | 緩い/安定 |
| `TH_POSE_COVARIANCE` | 強制更新閾値 | 厳格 | 緩い |

### NDTパラメータ

| パラメータ | 影響 | 小さい値 | 大きい値 |
|-----------|------|---------|---------|
| `VOXEL_SIZE` | 計算負荷/精度 | 重い/高精度 | 軽い/低精度 |
| `VOXEL_SIZE_MAP` | 計算負荷/地図精度 | 重い/高精度 | 軽い/低精度 |
| `LIMIT_RANGE` | 計算負荷/マッチング範囲 | 軽い/狭い | 重い/広い |
| `RESOLUTION` | NDT精度 | 細密/重い | 大局的/軽い |
| `MAX_ITERATION` | 計算負荷/収束性 | 速い/収束しにくい | 遅い/収束しやすい |
| `STEP_SIZE` | 最適化速度 | 慎重/遅い | 大胆/速い |
| `TRANS_EPSILON` | 収束判定 | 厳格 | 緩い |

---

## クイックリファレンス

### よくある問題と即効対策

| 問題 | 第一対策 | パラメータ |
|------|---------|-----------|
| CPU使用率高い | ダウンサンプリング | `VOXEL_SIZE: 0.5` |
| 位置がジャンプ | 閾値緩和 | `TH_MAHALANOBIS: 3.0` |
| NDT収束しない | 繰り返し増 | `MAX_ITERATION: 50` |
| FitnessScore高い | ボクセル調整 | `RESOLUTION: 1.0` |
| GPS不安定 | 閾値緩和 | `max_covariance_threshold: 15.0` |
| 特徴少ない環境 | 広範囲/大ボクセル | `LIMIT_RANGE: 60.0`, `RESOLUTION: 2.5` |

---

**推奨される調整手順:**
1. まずデフォルト設定で動作確認
2. 計算負荷が問題なら軽量化パラメータを適用
3. NDT精度が不足なら環境に応じてRESOLUTION/LIMIT_RANGEを調整
4. EKF精度が不足ならセンサ信頼度を調整
5. 環境に応じて閾値を微調整
</artifact>

上記のアーティファクトの右上にあるダウンロードボタンをクリックすることで、`EKF_manual.md`ファイルとしてダウンロードできます。

あるいは、内容を全選択（Ctrl+A / Cmd+A）してコピーし、任意のテキストエディタ（VS Code、Notepad++、メモ帳など）に貼り付けて、`.md`拡張子で保存してください。