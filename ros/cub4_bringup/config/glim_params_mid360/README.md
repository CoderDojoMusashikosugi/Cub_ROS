# Cub4 MID360 + 内蔵IMU用 GLIM設定

`glim_mid360.launch.py`から使用する設定一式です。
Velodyne版と同じ二段階処理（先頭区間でENU整合、その後GNSS付き地図生成）を使います。
描画なしでrosbagを直接読み込み、GPUでオドメトリとグローバル最適化を実行します。

```bash
./docker/run_in_container.sh 'cbs cub4_bringup'
./docker/run_in_container.sh 'ros2 launch cub4_bringup glim_mid360.launch.py rosbag:=/home/cub/rosbag2_2026_10_17-15_09_06_no_images'
```

先頭180秒の動作確認には `playback_duration:=180`、GNSSを使わない場合は
`enable_gnss:=false` を追加します。出力は既定で
`~/maps/<bag名>_glim_mid360_<日時>/` です。`dump_path`、`config_path`、
`start_offset`、`calibration_duration`、`gnss_config`もVelodyne版と同じです。

## 入力と時刻

| 項目 | 設定 |
|---|---|
| 点群 | `/livox/livox/lidar` (`sensor_msgs/msg/PointCloud2`) |
| IMU | `/livox/livox/imu` (`sensor_msgs/msg/Imu`) |
| 点群座標 | `livox_frame` |
| 内蔵IMU座標 | `livox_imu`（GLIM内で区別する名前） |
| GNSS | `/um982_node/fix_secondary`、RTK FIXのみ |

対象bagで点群約10 Hz・IMU約200 Hzを確認しています。
点群の`timestamp`はFLOAT64の**絶対ナノ秒**です。
`perpoint_relative_time=false`, `perpoint_time_scale=1e-9`を明示し、
最初の点の時刻をフレーム時刻として歪み補正に使用します。
レーザー番号は`line`、強度は`intensity`です。CustomMsgへの変換は不要です。

点群headerはスキャン先頭、記録は約0.10秒後です。これはスキャンの蓄積・配信時間で、
時刻補正として加えません。先頭のIMUと点群は同じ時刻系にあり、
`points_time_offset=0.0`, `imu_time_offset=0.0`とします。
Velodyneで使用した-1秒の補正を引き継いでいません。

内蔵IMUの静止加速度ノルムは約1で、ドライバも加速度を無変換で配信するため、
`acc_scale=9.80665`でgからm/s²へ変換します。
角速度はドライバのrad/sを使用します。別のbag・ドライバでは単位と時刻を再確認してください。

## 外部パラメータとGNSSアンテナ位置

内蔵IMUとLiDARの軸を平行とし、`T_lidar_imu`を次の値で初期設定しています。

```
xyz = [0.011, 0.02329, -0.04412] m
quaternion xyzw = [0, 0, 0, 1]
```

これは[FAST-LIOのMID360設定](https://github.com/hku-mars/FAST_LIO/blob/main/config/mid360.yaml)
にあるLiDAR→IMU変換を逆変換した値です。
GLIMの規約は `p_lidar = T_lidar_imu * p_imu` です。
実機の個別校正値ではなく、内蔵センサ間の公称値に基づく初期設定です。

bagの点群・IMUは両方とも`livox_frame`というheader名ですが、原点が異なるため
GLIMのIMU名を`livox_imu`としています。これによりIMU→LiDARの自己TFを避けます。
オフライン処理にrobot_state_publisherや追加TFは不要です。

`cub_description/urdf/cub4.urdf`から以下を使用します。

* `p_base_lidar = [-0.0379, 0, 0.5779] m`、pitch = **0.7854 rad**
* 右側サブアンテナ `p_base_antenna = [-0.12, -0.10, 0.5857] m`
* `p_imu_antenna = R_base_lidar.transpose() * (p_base_antenna - p_base_lidar) - p_lidar_imu`
* 結果 `[-0.0745688031, -0.12329, -0.0084181506] m`

GNSS原点は `[36.082628231, 140.076144739, 67.078]`、標高→楕円体高の補正は
`+40.1473 m`を引き継ぎます。原点・ノイズ・レバーアームなどは
`config_cub_gnss.json`で変更できます。

## 共通設定

GPU・サブマップ・点群削減設定はVelodyne版を初期値として保持しています。
GNSSの`information_scale=1000`も初期値を引き継いでおり、MID360の独立精度評価で
同定した値ではありません。点群数や環境を変更するときは重みを再評価してください。
IMUノイズもGLIM既定値で、Allan分散などによる実機同定は未実施です。

GNSS使用時は局所LIOに内蔵IMUを使い、グローバル側はVGICP、隣接LIO相対姿勢、
GNSSファクタで最適化します。設定の実行時コピー、保存・再読み込み、診断出力の詳細は
[GNSS拡張の説明](../../../cub_mapping/doc/glim_gnss.md)を参照してください。

## 実行確認（2026-10-30）

対象bag `rosbag2_2026_10_17-15_09_06_no_images`を、このlaunchから全区間処理し、
正常終了・地図保存を確認しました。GNSSなしのVelodyne版も10秒再生し、
launch処理の共通化後に動作することを確認しています。

* MID360の180秒試験：168 GNSSファクタ、GNSS残差RMSE 0.0577 m。
* 全区間：約2,109秒、21,079軌跡フレーム、77サブマップ、約153万保存点。
* 全区間のGNSSファクタ：1,792個。別プロセスで全個の復元を確認。
* GNSS残差：3D RMSE 0.0904 m、95パーセンタイル0.1485 m、最大0.3943 m。
* 軌跡の時刻逆転なし。隣接フレームの最大移動量0.537 m（時間間隔は一定ではありません）。
* 非RTK FIXの2,052メッセージは除外。間引き後16観測はサブマップ区間に対応せず不採用。

一部区間でIMUデータ間隔の空きや、点群とIMUの時刻差の警告が出ています。
GNSS残差は融合に使用した観測との整合性であり、独立した地図の絶対精度ではありません。
全区間の幾何精度・センサ外部パラメータの実測校正は未検証です。

検証出力はコンテナ内`/home/cub/glim_velodyne_work/mid360_gnss_full/`
（ホスト`docker/home/glim_velodyne_work/mid360_gnss_full/`）にあります。
GLIM dumpに加えて`map.pcd`、`map.ply`、`gnss_validation.png`、
`validation_summary.json`を保存しています。
