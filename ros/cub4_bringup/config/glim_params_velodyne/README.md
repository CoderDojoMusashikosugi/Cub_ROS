# Cub4 Velodyne + Spresense IMU 用 GLIM 設定

`rosbag2_2026_10_17-15_09_06_no_images` 向け。GLIM/GLIM ROS2 のコンテナ内にある設定を基に、使用する設定一式をこのディレクトリに保存しています。

- 入力: `/velodyne_points` (`velodyne`) と `/imu/data_raw` (`imu_link`)。
- Livox、OAK、車輪オドメトリは使用しません。画像入力先は bag に存在しない `/glim/unused_image` です。
- launchでは既定でサブアンテナRTK FIXを統合します。先頭60秒でENUへ初期整合した後、指定区間を再処理します。従来のGNSSなし処理は `enable_gnss:=false` で実行できます。[GNSS設定と出力](../../../cub_mapping/doc/glim_gnss.md)を参照してください。
- GPU版オドメトリ・グローバルマッピングを使用します。CUDA対応のGLIMとNVIDIA GPUが必要です。
- GUIを起動せず、bagを直接読み込みます。処理後に残りの最適化を待ち、GLIMのdumpを保存して終了します。

## センサ外部パラメータ

`cub_description/urdf/cub4.urdf` の `base_to_velodyne` と `base_to_imu` から、
`T_lidar_imu = inverse(T_base_velodyne) * T_base_imu` を計算しています。

- Velodyne: xyz = `[0, 0, 0.2492]`, rpy = `[0, 0, 0]`
- IMU: xyz = `[-0.030, 0, 0.2034]`, rpy = `[3.14, 0, 0]`
- `T_lidar_imu`: xyz = `[-0.030, 0, -0.0458]`, quaternion xyzw = `[0.9999996829, 0, 0, 0.0007963267]`

GLIMの変換方向は `p_lidar = T_lidar_imu * p_imu` です。
URDFに合わせてrollは正確なπではなく `3.14` を使用しています。
IMU加速度はbag内で約9.75 m/s²のノルムを持つため、`acc_scale = 1.0` です。
IMU雑音はGLIM既定値であり、このbagから同定した値ではありません。

## 時刻

このbagの複数区間（先頭、60、300、900、1500、2000秒付近）で、Velodyneのheader時刻は記録時刻より約1秒先、Spresense IMUは記録時刻より約5 ms前でした。
この一定差に対して `config_ros.json` の `points_time_offset = -1.0`、`imu_time_offset = 0.0` を設定しています。
これは記録時刻との比較に基づく補正で、センサの運動を使った高精度な時刻校正ではありません。別bagでは差を確認して調整してください。

Velodyneの `time` フィールドはFLOAT32の相対秒で、先頭点はheader時刻の約0.1001秒前です。
`perpoint_relative_time = true`, `perpoint_time_scale = 1.0`, `autoconf_prefer_frame_time = false` により、GLIMが負の点時刻を正に移し、同じ量だけフレーム時刻も移して歪み補正します。
点時刻を捨てる `global_shutter_lidar` は無効です。

## 主な設定

- 前処理: 距離1–100 m、0.25 mグリッド、目標10,000点/scan。
- 初期化: `LOOSE`、3秒。
- オドメトリ: GPU VGICP、5秒の固定遅延平滑化。
- サブマップ: 最大15キーフレーム、保存点群0.1 m、目標20,000点。
- グローバルマッピング: GPU VGICPによる最適化と暗黙のループ閉じ込み。
- 8 GB GPUで長時間bagを処理するため、サブマップの点数を制限し、グローバル最適化では点群の20%を使用します。保存する地図にはサブマップの全点を使用します。
- 描画なし。`config_viewer.json` は保存後のoffline viewer用に含めています。

## GNSS導入前の実行確認

対象bag全体を約11分で処理し、正常終了と保存を確認しました。
443サブマップ、20,925軌跡フレーム、約2.27 km、保存点群8,860,000点です。
初期化に使う先頭約3秒を除いて末尾まで軌跡があり、時刻の逆転はありません。
終盤のGPUメモリ使用量の観測値は約2.4 GBでした（厳密なピーク測定ではありません）。

GLIMのIMU予測検証（主に並進）と終盤の点群・IMU時刻差の警告は残っています。
正常終了は地図の絶対精度や高さ精度を保証するものではなく、実測との比較やIMU雑音・バイアス・外部パラメータの精密校正は未実施です。

GLIMの設定仕様: <https://koide3.github.io/glim/parameters.html>
