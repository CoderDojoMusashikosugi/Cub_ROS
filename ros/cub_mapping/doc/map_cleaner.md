# MapCleanerによる動体除去

CubのROS2/GLIM環境でスキャンを保存し、別のROS Noetic Dockerで
[MapCleaner_Unofficial](https://github.com/kamibukuro5656/MapCleaner_Unofficial)を実行します。
ROS1への移植やros1_bridgeは不要です。Docker内のROS masterを外部ネットワークに公開しません。

## 必要な環境

* 現在のCub ROS2コンテナ、GPU対応GLIM、元のrosbag。
* ホストでDockerを実行できること。Python 3、`numpy`、`yaml`、Git。
  Ubuntuのホストに不足している場合のパッケージ名は`python3-numpy python3-yaml git`です。
* 初回セットアップ用ネットワーク接続。Noeticイメージと保存スキャン・出力用のディスク空き。
* 以下のコマンドはホストの`/home/tomoki-na/Cub_ROS`で実行します。

## 1. 初回セットアップ

```bash
cd /home/tomoki-na/Cub_ROS
./scripts/setup_map_cleaner.sh
```

このスクリプトは次を行います。

1. MapCleanerをコミット`34d1052c35de11bef6bfe5dbef8bde92b5dea2a8`に固定して取得。
2. `cub-map-cleaner:34d1052`イメージをビルド。
3. 既存Cubコンテナで`cub_mapping`と`cub4_bringup`をビルド。

取得コードとDocker build contextは`docker/home/.cache/map_cleaner/`に配置します。
既に同じ版があれば再利用し、変更されたcheckoutには上書きしません。
Noetic本体だけを作り直すときは`--skip-ros-build`を使えます。

上流の動体除去処理は変更していません。Docker build時に、PCD書き込み失敗を
エラーにする変更と、処理後のROS待機を終了する変更だけを適用します。
RVizを起動しない専用batch launchを使います。上流BSDライセンスは
`docker/map_cleaner/UPSTREAM_LICENSE`とイメージ内ソースに保持しています。

## 2. GLIMでスキャン付き地図を生成

MID360の例です。`dump_path`には未使用の出力先を指定します。

```bash
./docker/run_in_container.sh \
  'ros2 launch cub4_bringup glim_mid360.launch.py \
    rosbag:=/home/cub/rosbag2_2026_10_17-15_09_06_no_images \
    dump_path:=/home/cub/maps/mid360_for_cleaner \
    save_frames:=true'
```

シェルの引用内の改行・バックスラッシュが分かりにくい場合は、1行でも実行できます。

```bash
./docker/run_in_container.sh 'ros2 launch cub4_bringup glim_mid360.launch.py rosbag:=/home/cub/rosbag2_2026_10_17-15_09_06_no_images dump_path:=/home/cub/maps/mid360_for_cleaner save_frames:=true'
```

既定ではGNSS付きです。初期整合パスではスキャンを保存せず、本番パスだけで保存します。
`enable_gnss:=false save_frames:=true`の組合せも可能ですが、その場合は元のLIO座標です。
短い生成試験には`playback_duration:=180`を追加できます。

出力構成:

```text
docker/home/maps/mid360_for_cleaner/
├── traj_lidar.txt
├── gnss_report.json
├── frames/
│   ├── manifest.csv
│   ├── report.json
│   ├── 00000000/
│   │   ├── points_compact.bin
│   │   └── intensities_compact.bin
│   └── 00000001/ …
└── その他のGLIM dump
```

保存対象は**GLIMが前処理・歪み補正を済ませた点群**です。IMU座標の点群をLiDAR座標へ
戻し、`traj_lidar.txt`と組み合わせられるようにします。二重の歪み補正はしません。
`keep_raw_points`をtrueにしても、この保存プラグインの点数は増えません。

書き込みは同期処理なので、終了時の未保存キューはありません。GLIMの保存後に
最終軌跡の時刻で照合し、軌跡の各行と同じ番号へ並べ直します。対応するスキャンが
ない場合や書き込みが失敗した場合は成功レポートを出さず、launchを失敗させます。
GLIMのフレームIDをそのまま軌跡の行番号とは扱いません。
オドメトリが通知した最終スキャンが地図の最終軌跡に含まれない場合、その末尾分は
`frames/unreferenced_tail/`に保存し、MapCleanerには渡しません。件数は`report.json`に記録します。

## 3. 最初の1,000フレームで試験

```bash
./scripts/run_map_cleaner.sh \
  --input docker/home/maps/mid360_for_cleaner \
  --output docker/home/maps/mid360_cleaner_test \
  --sensor mid360 --start 0 --end 1000
```

`start`は含む、`end`は含まないフレーム番号です。1,000フレームは約100秒に相当します。
入力データには同じ実行の`frames/`と`traj_lidar.txt`が必要です。
スクリプトが時刻・点数・ファイルサイズ・センサ種別を検査してからDockerを起動します。
入力は読み取り専用でマウントし、出力は実行ユーザーの所有ファイルとして作ります。

MapCleanerのパラメータは以下にあります。

* `ros/cub_mapping/params/map_cleaner_mid360.yaml`
* `ros/cub_mapping/params/map_cleaner_velodyne.yaml`

`--config /path/to/custom.yaml`で別ファイルを指定できます。入力・出力先とフレーム区間、
batch実行用の設定だけはCLIから生成します。その他の設定は指定YAMLを保持します。
`--threads 4`が既定値です。

MID360は非反復走査のため、角度セルと観測の疎密を考慮して結果を評価してください。
MID360プリセットの垂直視野120度は、非対称な実視野を対称レンジ画像に収めるための
保守的な窓です。センサの公称垂直視野が120度という意味ではありません。
未観測セルは上流実装で未知扱いになります。
地面抽出は姿勢で鉛直方向をそろえてから行います。

メモリと重複点を抑えるため、初期設定では地面抽出時に0.15m voxel filterを使い、
3フレームに1回を採用します。動体判定も3フレームに1回です。
間引きを変更する場合は両方の`frame_skip`を確認してください。

## 4. 結果の確認と全区間処理

```bash
ls -lh docker/home/maps/mid360_cleaner_test/*.pcd
```

CloudCompareで以下を開き、地面・壁・柱などを誤除去していないか確認します。

| ファイル | 用途 |
|---|---|
| `ground.pcd` | 地面として残す点 |
| `static.pcd` | 地面より上の静止物として残す点 |
| `dynamic.pcd` | 動体として除去する候補 |
| `ground_below.pcd` | 推定地面より下の点 |
| `other.pcd` | 処理対象外の点。既定では0点の場合もあります |
| `terrain.pcd` | 推定した地表面モデル |

`config.yaml`は実際に使用した設定、`processing.log`は処理ログ、
`run_report.json`は入力・版・採用フレーム・各出力点数・座標範囲です。
ROSプロセスの終了コードだけでなく、全PCDの存在・点数・バイナリサイズ・有限値も検査します。
失敗したジョブの`run_report.json`は`success: false`になります。
batch終了時にroslaunchが`REQUIRED process ... has died!`と表示することがあります。
併記された`process has finished cleanly`は正常終了を示します。最終的な成否は
スクリプトの終了コードと`run_report.json`で確認してください。
空でない出力先は再利用できません。再試験には別の出力名を指定してください。

全区間:

```bash
./scripts/run_map_cleaner.sh \
  --input docker/home/maps/mid360_for_cleaner \
  --output docker/home/maps/mid360_cleaned \
  --sensor mid360 --start 0 --end -1
```

MapCleanerの設定だけを変えるときはGLIMを再実行する必要はありません。
ただし、GLIMの最適化・センサ設定・原点を変える場合は対応する入力を再生成してください。

## 5. PCD・PLY・XYZへ出力

```bash
./scripts/export_clean_map.sh \
  --ground docker/home/maps/mid360_cleaned/ground.pcd \
  --static docker/home/maps/mid360_cleaned/static.pcd \
  --output docker/home/maps/mid360_cleaned/clean_map
```

`clean_map.pcd`、`clean_map.ply`、`clean_map.xyz`、`clean_map.json`が生成されます。
XYZ列は`X Y Z intensity`です。地面と静止物を連結し、中心化・回転・単位変換・
地面へのクランプは行いません。`run_report.json`の原点情報も出力JSONへ引き継ぎます。
`ground_below`、`dynamic`、`other`は自動結合しません。
同名出力がある場合は上書きせずエラーにします。

全点について、PCD・PLYの値が結合前と完全一致し、XYZの丸め誤差が
5.1e-10以内であることを別のPython読込処理で検査できます。

```bash
python3 scripts/tests/verify_clean_map.py \
  --prefix docker/home/maps/mid360_cleaned/clean_map \
  --ground docker/home/maps/mid360_cleaned/ground.pcd \
  --static docker/home/maps/mid360_cleaned/static.pcd
```

出力スクリプトはCubコンテナ内のPCLを使うため、入力・出力にはホストの
`docker/home/`または`ros/`以下のパスを指定します。MapCleanerのrun自体は任意の
ホストパスを使用できます（Dockerのmount構文により、カンマを含むパスは不可）。

Unrealへの取り込みでは`Auto Center On Import`を無効にし、原点・m→cm換算を
確認してください。MapCleanerはGNSS基準やGoogleメッシュとの高度差を補正しません。

## Velodyneで使用する場合

地図生成コマンドを`glim_velodyne.launch.py`にし、MapCleanerを
`--sensor velodyne`で実行します。異なるセンサの設定を選ぶと入力検査で拒否します。

## 開発用テスト

```bash
python3 -m unittest discover -s scripts/tests -v
./docker/run_in_container.sh 'cd /home/cub/colcon_ws && colcon test --packages-select cub_mapping --ctest-args -R "test_gnss|test_save_frames" --output-on-failure'
```

保存・変換のテストは座標維持とファイル対応を検証します。動体除去の正解ラベル付き
精度評価ではありません。実データでは`dynamic.pcd`の誤除去確認を行ってください。
