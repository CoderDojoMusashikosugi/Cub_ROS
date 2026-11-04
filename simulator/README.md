# Cub Simulator

Godot 4.7.2で動作確認した、ROS接続・オンライン検証用シミュレーターです。ホストの主ロボットはROSの速度指令だけで動作し、クライアントの通行人・コーン・別ロボットはGodot内で操作します。同じプロジェクトをホストまたはクライアントとして起動し、ロボット・通行人・コーンを同じ空間で操作できます。

ホストが操作する主ロボットは、`ros/cub_description`のCub4モデルをURDFの縮尺・向き・色で表示します。クライアントから見た主ロボットもCub4です。他のロボット・通行人・コーンは従来の簡易モデルです。表示モデルの由来と再生成手順は[Cub4モデル](assets/cub4/README.md)を参照してください。

## ROSを通した主ロボットの操作

主ロボットの制御経路は常に次のとおりです。GodotはROS未接続時や`/cmd_vel_atom`の受信が0.3秒途絶えたときは停止します。WASDから主ロボットを直接動かす経路はありません。

```text
WASD / DualSense / Xbox → GodotでDualSense形式のJoyへ変換
 → rosbridge → /joy → cub_commander → /cmd_vel_atom → Godotの主ロボット
```

リポジトリのルートから、初回に依存パッケージを導入してビルドします。Dockerイメージを再構築する場合は`cub_simulation/package.xml`の依存がrosdepで導入されます。

```bash
./docker/run_in_container.sh 'sudo apt-get update && sudo apt-get install -y ros-jazzy-rosbridge-server'
./docker/run_in_container.sh 'cd /home/cub/colcon_ws && colcon build --symlink-install --packages-select cub_commander cub_simulation cub_bringup'
./docker/run_in_container.sh ros2 launch cub_bringup simulation.launch.py simulation:=godot
```

実機の`launch_at_boot.launch.py`は起動しません。sim用launchは`cub_commander`とrosbridgeを起動し、`joy_linux_node`・micro-ROS agent・実機センサードライバーを起動しません。`/joy`・`/cmd_vel`・`/cmd_vel_atom`の名前は実機と共通です。Godotから送るJoyの配置は接続したコントローラーの機種によらず既存DualSense形式にそろえます。commanderはコミット済みの実装をそのまま使い、ROS側で機種を判別できない場合のDualSense扱いを利用します。ROS側の`/dev/input/js0`にXboxが接続されている場合は、形式の判別がGodotの送信形式と一致しません。

macOSのVNCコンテナではホストの9090番ポートを同一PC・別PCから接続できるよう公開します。既存コンテナにはComposeのポート設定変更を反映するため再作成が必要です。別PCのROSへ接続する場合は、以下のROS URLを`ws://Dockerを動かしているPCのIPアドレス:9090`へ変更してください。

GodotでHostを選びます。開始前の`Host ROS URL`の既定値は`ws://127.0.0.1:9090`です。実行引数を使う場合の例（Godotの実行ファイルの指定方法はOSに応じて変更）:

```bash
# macOS。Windows/Linuxでも -- 以降のアプリ用引数は共通。
/Applications/Godot.app/Contents/MacOS/Godot --path simulator -- --host --ros-url ws://127.0.0.1:9090
```

入力元は自動選択します。Godotで認識したDualSense・Xbox Series X|Sが接続されていればコントローラーを使い、接続されていなければWASD・矢印キーを使います。接続・切断は実行中にも反映され、現在の入力機器名を下部に表示します。クライアントはROSへ接続しません。

| 入力 | 手動・自律の切り替え |
| --- | --- |
| WASD・矢印キー | 押している間は手動。Wは前進、Aは左旋回。何も押さなければ自律指令を使用 |
| Space | 単独で押すと手動の速度0指令。WASD・矢印と同時に押した場合は移動キーが優先。すべて放すと自律へ戻る |
| DualSense・Xbox | L1/LBまたはL2/LTを押している間は左スティックで手動操縦。放すと自律へ戻る。接続しているだけでは手動にならない |

コントローラーが選ばれている間はキーボードのロボット操作を使いません。十字キーは既存commanderの速度設定変更、DualSenseのタッチパッド押下は既存の再開操作です。Xbox固有のShareボタンを再開操作には割り当てていません。初期速度はcommanderの1.2 m/s・60度/sを使い、Shift加速は主ロボットには適用しません。逆方向のキーを同時に押して移動量が0になった場合も、押している間は手動の速度0指令です。

`--joy-device ID`は接続中の指定IDを優先します。既定`-1`は自動選択です。複数台ある場合は現在の機器を維持し、それが切断されたら別の認識済みコントローラー、なければキーボードへ切り替えます。入力配置が不明なコントローラーは選択しません。切り替え時には切断前の入力を保持せず、新しい入力元の現在値を使用します。

地図表示中・TPSのEscによるカーソル解放中もROS用入力を送ります。フォーカス喪失を理由に手動停止を送る処理はなく、コントローラーの入力はバックグラウンドでも中継します。キーボードはGodot/OSから得られるキー状態を使うため、別アプリを操作中のキーを取得する機能ではありません。キーが解放された状態では自律モードになります。自律走行には、別ノードから継続して`/cmd_vel`を送る必要があります。

commanderは従来どおりJoyの`header.stamp`から1秒の途絶を判定し、手動を解除して自律へ戻ります。Godotはheaderを省略して送信し、ROS側のrosbridgeがROS時刻を補完します。自律モードで`/cmd_vel`が3秒途絶えると速度0を送ります。手動解除時は次の自律指令が届くまで直前の手動指令が残る場合があるため、自律指令は継続送信する前提です。Godot側のWebSocket切断時は速度を消去し、再接続後は新しい指令を待ちます。下部の`ROS CONNECTING`／`ROS WAIT CMD`／`ROS OK`と入力機器名で接続状態を確認できます。Godot側の速度受信上限は前後5 m/s・旋回2π rad/sです。

### シミュレーション種類とcommon

launch引数`simulation`は`none`が既定で実機用の動作を維持します。現在の対応値は`none`と`godot`です。未対応値は起動エラーになります。launch内の起動条件と追加方法は[ROS側のREADME](../ros/cub_simulation/README.md)を参照してください。

```bash
# 必要な場合だけ別ターミナルで起動。cub4_bringupもビルドしておく。
./docker/run_in_container.sh ros2 launch cub_bringup common.launch.py simulation:=godot
```

Godot指定ではCub4のcommonを選び、wheel odometry・実機の再開ボタン・オドメトリクリア・GNSS状態監視を停止します。機体の既定設定`CUB_TARGET`自体は変更しません。`cub4_bringup`が機体設定による`COLCON_IGNORE`対象の場合は、commonの使用前にそのパッケージを有効にしてビルドしてください。距離ログ・URDFの固定TF・システム診断は継続し、トピック監視はJoyと速度指令に切り替えます。

Joy/Twistの往復、簡易LiDAR（`/scan`）、およびシミュレータ座標に基づく真値オドメトリ（`/odom`、20 Hz）を配信します。簡易LiDARの位置と`frame_id`は`ros/cub_description/urdf/cub4.urdf`の`base_to_velodyne`（`velodyne`、`base_link`から高さ0.2492 m）を参照し、10 Hz・360本で壁・箱・他のロボット・通行人・コーンのセンサー用形状を測距して`/scan`へ配信します。床・坂道・階段・凹凸地形はLiDAR検出から除外し、床と壁が一体の三角形メッシュでは面の法線から壁面だけを抽出します。`navigation.launch.py simulation:=godot map:=/home/cub/colcon_ws/src/cub/cub_simulation/maps/cub_playground_v1/map.yaml`で実機と同様に地図パスを引数で渡してNav2自律移動を実行できます（内部の`localization.launch.py`では`odom_to_tf`で`odom -> base_link`を配信します）。`simulation`とROS時刻の`use_sim_time`は別で、現在は実時間（`false`）です。

## Godotエディタから役割を指定して起動する

「デバッグ → 実行インスタンスのカスタマイズ（Customize Run Instances）」を開き、複数インスタンスを有効にして数を**4**に設定します。「メインの実行引数（Main Run Args）」を空にし、各行の「起動引数（Launch Arguments）」へ次を入力して、`F5`で実行します。

| インスタンス | 起動引数（そのまま貼り付け） |
| --- | --- |
| 1：サーバー・主ロボット・TPS | `-- --host --actor robot --camera tps --name Host` |
| 2：クライアント・通行人・3D地図 | `-- --join 127.0.0.1 --actor pedestrian --camera 3d --name Walker` |
| 3：クライアント・別のロボット・TPS | `-- --join 127.0.0.1 --actor robot --camera tps --name Robot` |
| 4：クライアント・コーン・2D地図 | `-- --join 127.0.0.1 --actor cone --camera 2d --name Cone` |

先頭の`-- `はGodotの引数とアプリの引数を分けるために必要です。この例ではメインの実行引数を空にし、各行へ設定をまとめています。共通引数と個別引数を組み合わせる方法は[Godotの実行インスタンス設定](https://docs.godotengine.org/en/stable/tutorials/scripting/debug/overview_of_debugging_tools.html#customize-run-instances)を参照してください。

`Host`／`Join`ボタンを押さずに指定した役割で接続を開始し、接続後は下部1行のステータス表示へ切り替わります。別のLAN/VPNホストへ接続する場合は`127.0.0.1`をそのホストのアドレスに置き換えます。

| アプリの引数 | 設定内容・既定値 |
| --- | --- |
| `--host` | サーバーとして自動開始。`--join`との同時指定は不可 |
| `--join ADDRESS` | 指定したサーバーへ自動参加 |
| `--world WORLD_ID` | ホストのワールドID（例：`cub_playground_v1`、`gnss2026_mid360_1017_02`）。既定`cub_playground_v1`。`--join`との同時指定は不可 |
| `--spawn PRESET` | 自身の開始位置プリセット名（例：`start_a`）。`--spawn-x`／`--spawn-y`との同時指定は不可 |
| `--spawn-x X --spawn-y Y` | 自身の開始位置（ROS `map` 座標系、m単位）。`--spawn-x`と`--spawn-y`は両方指定が必要 |
| `--spawn-z Z` | 座標指定モードでの高さ（m単位、省略時はワールドの床高を自動解決） |
| `--spawn-yaw-deg ANGLE` | 座標指定モードでの向き（度単位、ROS `+X` が `0`、左旋回が正、省略時 `0`） |
| `--bind ADDRESS` | サーバーの待ち受け先。既定`127.0.0.1`。LAN/VPNから受け付ける場合は`0.0.0.0` |
| `--port NUMBER` | UDPポート。1024〜65535、既定27140。全インスタンスでそろえる |
| `--actor KIND` | クライアントの種類`robot`／`pedestrian`／`cone`。既定はクライアントで`pedestrian`、ロビーで`robot`。サーバーはこの指定によらず主ロボットに固定 |
| `--camera VIEW` | 初期視点`tps`／`3d`／`2d`。既定`tps`。`orbit`は`3d`、`top`は`2d`と同じ |
| `--name NAME` | 表示名。空白を含む場合は`--name "Other Robot"`。既定Host／Guest／Player |
| `--ros-url URL` | ホストのrosbridge接続先。既定`ws://127.0.0.1:9090` |
| `--joy-device ID` | 接続中に優先するGodotコントローラーID。既定`-1`は自動選択 |
| `--run-seconds NUMBER` | 指定した秒数で自動終了。省略または0で無効 |

`--camera=3d`のような`=`形式も使えます。同じ設定を複数回指定した場合は後の値を使います。`--host`／`--join`を省くと、ワールド・開始位置・キャラクター・名前・接続先・初期視点だけを設定して開始前の画面で待ちます。引数の値や役割の組み合わせ、範囲外・壁内部など配置不可の開始位置が指定された場合は、別位置へ黙って移動せず開始前の画面に理由を表示します。

## 手動で起動する

Godotで`project.godot`を開いて実行し、開始前の画面で名前・クライアント用アクター（`Client actor`）・ホストのワールド（`Host world`）・開始地点プリセット（`Start preset`）・接続先・ポートをマウスで設定します。ワールド選択と開始地点選択は文字の一覧のみで表示し、プレビュー画像は使いません。1つ目のウインドウで`Host`、残りで`Join`を選択します。クライアントは参加先ホストが選んだワールドを自動的に読み込み、衝突形状の準備完了後に参加します。`Host`はアクター選択にかかわらず主ロボットで開始します。同じPC内の接続先とホストの既定の待ち受け先は`127.0.0.1`です。ホストは最大3クライアントを受け付けます。接続中は`Cancel`で中断できます。

プレイ開始後は設定画面を隠し、下部の1行（高さ28px）に役割・キャラクター・視点・現在座標・向き・キャラクター数・FPSを表示します。プレイ中のボタンや操作説明は表示しません。退出・接続失敗・ホスト切断時は、マウスで操作できる開始前の画面に戻ります。

別PCから接続する場合は、ホストのアドレス欄に`0.0.0.0`を入力して`Host`を押すか、コマンドラインで`--bind 0.0.0.0`を指定します。クライアントではホストの実際のLAN/VPNアドレスを指定します。Windowsのファイアウォール確認が表示された場合も、同一PCの`127.0.0.1`接続にネットワークアクセスの許可操作は不要です。

コマンドラインでも起動できます。以下はWindows側のリポジトリルートからの例です。

```powershell
$godot = 'C:\Users\tomok\workspace\Godot_v4.7.2-stable_win64.exe\Godot_v4.7.2-stable_win64.exe'
& $godot --path .\simulator -- --host --name Host --actor robot
& $godot --path .\simulator -- --join 127.0.0.1 --name Walker --actor pedestrian
```

アプリ用の引数は`--`の後に置きます。`--actor`は`robot`・`pedestrian`・`cone`、`--port`は既定27140です。GodotのmacOS版・Linux版でも同じプロジェクト・アプリ用引数を使う構成です。OSごとの検証状況は「検証」を参照してください。

## 操作

TPSではカーソルをウインドウ内に捕捉し、ボタンを押さずにマウスを動かすだけで視点を変えます。マウスホイール・トラックパッドの2本指上下スクロール・ピンチで追従距離を変えられます。`Esc`でカーソルを解放して別ウインドウを選択でき、もう一度`Esc`を押すか操作画面を左クリックすると視点操作に戻ります。カーソル解放中やウインドウが非アクティブなときは視点操作とクライアントの移動入力を止めます。主ロボットへのROS用入力は引き続き送信します。

地図モードではカーソルを表示し、Google Mapsのように左ドラッグで地図をつかんで移動します。3D地図では右ドラッグ・中ドラッグ、または`Ctrl`／`Command`＋左ドラッグで回転・傾きを変えます。macOSのControl＋クリックにも対応します。左ドラッグ中に`Ctrl`／`Command`を押すと回転へ、離すと地図の平行移動へ切り替わります。Controlを押してドラッグを開始した場合も、ボタンを押したまま切り替えられます。トラックパッドでは`Ctrl`／`Command`を押しながら2本指スクロールすると回転・傾きを変えられます。真上の2D地図は向きを固定します。両方ともホイール・2本指上下スクロール・ピンチでカーソル位置を基準にズームし、ダブルクリックで拡大できます。ピンチは指を広げると拡大、狭めると縮小します。地図モードでもWASDでキャラクターを操作でき、地図の3D／2D切り替えでは見ている場所を保ちます。

| 操作 | キー・マウス |
| --- | --- |
| ロボットの前後移動 | W / S、または上下矢印 |
| ロボットの左右旋回 | A / D、または左右矢印 |
| 通行人・コーンの移動 | WASD / 矢印。カメラの水平向きが基準 |
| クライアントの速度を上げる | Shift。主ロボットの速度はcommanderで決定 |
| TPS追従視点 | 1 |
| 3D地図視点 | 2。カーソルを表示 |
| 真上の2D地図視点 | 3。カーソルを表示。この視点のキャラクター移動は画面の上下左右が基準 |
| TPSで周囲を見渡す | マウス移動。ボタン操作不要 |
| 3D地図の回転・傾き | 右／中ドラッグ、Ctrl／Command＋左ドラッグ、またはCtrl／Command＋2本指スクロール |
| カメラをズーム（全視点） | マウスホイール、2本指上下スクロール、ピンチ。地図ではカーソル位置が基準 |
| 地図を拡大 | 左ダブルクリック |
| 地図の平行移動 | 左ドラッグ。I / J / K / Lでも可能 |
| 自分の操作対象へカメラを戻す | F |
| クライアントのキャラクター変更 | T（ロボット → 通行人 → コーン）。ホストは主ロボットに固定 |
| TPSでカーソル解放・復帰を切り替え／地図のドラッグを中断 | Esc |
| TPSでカーソル解放中に視点操作へ復帰 | 操作画面を左クリック |
| 退出して開始前の画面へ戻る | Backspace |

キャラクター同士はすり抜けます。固定地形・壁には衝突します。全キャラクターは直立し、坂道や小段差へ接地します。ジャンプ、歩行アニメーション、自動移動はありません。

既定の試験ワールド（`cub_playground_v1`）には坂道、15cm刻みの階段、細かい三角形で作った凹凸地形、乗り越えられない高さの箱があります。ロボットも通行人も同じ接地・段差乗り越え処理を使います。初期の最大段差高は24cm、登れる傾斜は46度に設定しています。実機の走破性能の再現値ではありません。

### 2D占有地図（PGM）からのワールド生成と選択

`simulator/worlds/<world_id>/`に配置したワールド定義（`world.json`）から、試験ワールド（`cub_playground_v1`）と2D画像押し出しワールド（`gnss2026_mid360_1017_02`など）を選択して起動できます。

- `simulator/tools/build_occupancy_world.py`は、編集済みPGM画像とYAML（解像度・原点）からロスレスグレースケール`source.png`、64×64セル単位の結合壁矩形`wall_chunks.json`、および`world.json`を生成します。
- 黒（画素値`0`）のみを高さ`0.5 m`の壁として押し出し、白（`254`）と灰色（`205`等）はすべて平らな空きの床（`0.0 m`）として扱います。画像の膨張・平滑化・穴埋め・切り出しは行いません。
- 床には任意のPNG画像を`--floor-texture`で指定できます。PGMと同じキャンバスサイズ・座標配置の画像を使い、床全体に1枚として貼ります。最近傍補間・ミップマップなしで、拡大時もピクセルの四角と元の色を保ちます。`gnss2026_mid360_1017_02`には`map_intensity.png`を同梱済みです。画像は見た目にだけ使用し、壁やLiDARの判定はPGMが決めます。
- `gnss2026_mid360_1017_02`（`3580 × 2122` px、`0.2 m/cell`、`716.0 × 424.4 m`）の既定開始地点`start_a`は、画像上の`(u=773, v=1467)`（ROS `map` 座標 `(x=62.2725, y=30.194, z=0.0)`）で画像左向き（`-u`方向、`yaw = 180°`）です。`3`キーの真上2D地図カメラも元PGM画像の向き（画面上が画像上`-v`＝ROS `+Y`、画面右が画像右`+u`＝ROS `+X`）に一致します。
- Godotのワールド選択はROSのナビゲーション地図（`map:=...`）とは独立しており、ワールド変更に伴うROS側の地図切替・一致照合・Nav2再起動は行いません。

#### 手動でPGMマップを編集・生成してワールドを追加する手順

1. **PGM画像の編集・作成（GIMP等の画像編集ソフト）**
   - **黒（画素値`0`）**：高さ`0.5 m`の壁（移動の衝突＋LiDAR反射対象）になります。境界がぼけて灰色（`> 0`）になると空き床として扱われるため、アンチエイリアスのない鉛筆ツール（ハードエッジ・値`0`）で描きます。
   - **白（`254`〜`255`）・灰色（`205`など、`1`以上の値）**：すべて空きの平らな床（`0.0 m`）になります。
   - 既存のROS `map.yaml`と座標系を一致させる場合は、**画像のキャンバスサイズ（幅×高さ）の変更・トリミング・拡大縮小を行わず**、そのままの寸法で保存します。
   - 書き出し形式は**8-bitグレースケールPGM（P5 Raw形式推奨、P2 ASCII形式も可）**を選択します。
   - スタート地点にしたい場所のピクセル座標 `(u, v)`（左上が`(0, 0)`、横が`u`、縦が`v`）を画像編集ソフト上で確認します。

2. **変換スクリプトの実行**
   - リポジトリのルートで`simulator/tools/build_occupancy_world.py`を実行すると、`simulator/worlds/<world-id>/`に`world.json`・`source.png`・`wall_chunks.json`が生成されます（Python標準ライブラリのみで動作します）。
   - 向きの指定（`--spawn-yaw-deg`）は、`0`＝画像右向き（`+u`／ROS `+X`）、`90`＝画像上向き（`-v`／ROS `+Y`）、`180`＝画像左向き（`-u`／ROS `-X`）、`-90`（または`270`）＝画像下向き（`+v`／ROS `-Y`）です。

```bash
# 例1：既存の map.yaml から解像度(resolution)と原点(origin)を引き継いで生成する場合
python3 simulator/tools/build_occupancy_world.py \
  --pgm docker/home/maps/gnss2026/mid360/1017_02/map_navigation.pgm \
  --yaml docker/home/maps/gnss2026/mid360/1017_02/map.yaml \
  --floor-texture docker/home/maps/gnss2026/mid360/1017_02/map_intensity.png \
  --world-id gnss2026_mid360_1017_02 \
  --display-name "GNSS 2026 Mid360 1017_02" \
  --spawn-pixel 773 1467 \
  --spawn-yaw-deg 180

# 例2：YAMLを使わず、解像度（m/cell）と原点（ROS座標 X Y YAW_RAD）を直接指定する場合
python3 simulator/tools/build_occupancy_world.py \
  --pgm /path/to/custom_map.pgm \
  --resolution 0.2 \
  --origin -50.0 -50.0 0.0 \
  --world-id custom_test_01 \
  --display-name "Custom Test 01" \
  --spawn-pixel 250 250 \
  --spawn-yaw-deg 0
```

`--floor-texture`を省略すると従来の単色床になります。指定したPNGは変更せず`floor_texture.png`としてワールド内にコピーされ、`world.json`の`appearance.floor_texture`と`floor_texture_sha256`に記録されます。元の`docker/home/maps/`は実行時には不要です。画像の寸法がPGMと異なる場合は、位置ずれを防ぐため生成をエラーにします。床画像もGodot同士の`world_hash`照合対象なので、別PCには更新したワールドフォルダ一式を配布してください。ROSのナビゲーション地図には影響しません。

3. **Godotでの選択・開始**
   - `simulator/worlds/<world-id>/world.json`が配置されていれば、Godot起動時に自動で検出され、開始前のロビー画面の`Host world`一覧に表示されます（スクリプトの変更は不要です）。
   - ロビーの`Host world`と`Start preset`を選んで`Host`を押すか、起動引数`-- --host --world <world-id> --spawn start_a`（または任意座標`--spawn-x <X> --spawn-y <Y> --spawn-yaw-deg <DEG>`）で開始できます。
   - 複数の開始地点プリセットを用意したい場合は、生成された`world.json`の`"spawns"`に`"start_b"`などを追記するとロビーの`Start preset`一覧に追加されます。
   - オンライン参加する別PCのクライアントにも、同じ`simulator/worlds/<world-id>/`フォルダ一式を配置しておけば、ホストの選択に合わせて自動で読み込まれます。

## 軽量化と構成

- Compatibility / OpenGLレンダラー。影・MSAA・ポストエフェクトを使いません。
- 主ロボットは変換済みのCub4モデルを同梱し、1メッシュ・1サーフェス、頂点色、LODで描画します。テクスチャは使いません。他のアクターとワールドは単純な形状です。起動時のOBJ・URDF変換は不要です。
- 2D画像押し出しワールドは黒セルを矩形結合し、64×64セル（12.8 m四方）のチャンク単位でメッシュと静的衝突形状（壁は移動・LiDAR共通、床は移動専用でLiDAR対象外）を構築します。壁同士の内部境界面は省略してチャンク境界の引っかかりを防ぎます。
- 描画上限は操作中45 FPS、フォーカスが外れたウインドウは30 FPSです。接地・移動の更新は60 Hzです。
- 位置・向きの通信は20 Hz。参加・退出・種類変更は信頼性のあるRPC、移動更新は最新値を優先するRPCで送ります。
- 操作担当だけが移動を計算し、位置・向きに操作担当のシミュレーション経過時刻を付けて送ります。ホストはその時刻を保持して中継します。
- 遠隔キャラクターは約120 msの位置履歴を使って描画時に補間し、まとめて届く更新や軽いパケット欠落による速度の変動を抑えます。重複した状態で再生時刻を更新せず、自分の操作対象は即座に動かします。端末間の時計同期は不要です。
- 受信が途切れたときは最後の位置で止め、将来位置への外挿は行いません。大きな位置変更は即座に反映し、復帰時に長い表示遅延をためません。物理状態の再現や巻き戻しは行いません。
- 省電力モードを使い、Godotのワーカースレッド数を1プロセスあたり2に制限します。
- ホストは常に主ロボットの移動処理を保持し、他の参加者の状態を中継します。
- ホストが選んだ`world_id`・`world_hash`・通信プロトコルの版（版3）を照合し、クライアント側で該当ワールドの読み込みと衝突形状の準備が完了してからアクターを生成・同期します。

`scripts/session.gd`が通信、`scripts/actor.gd`が共通移動、`scripts/pose_buffer.gd`が遠隔表示の補間、`scripts/camera_rig.gd`がカメラ、`scripts/world.gd`・`scripts/world_registry.gd`・`scripts/occupancy_loader.gd`がワールド管理と2D占有ワールド生成です。現在は少数のRPCと状態配列で同期し、ノードの自動複製機構は使用していません。ホストの状態記録は受信した最新の正解位置を保持し、描画だけを遅らせます。

ワールド合意と時刻付き通信のプロトコルは版3です。ホストとクライアントで同じプロトコル版・ワールド定義を使用します。

Godotの生成データと検証ログは`.godot/`以下に保存し、Gitでは無視します。

## 検証

Godotを使って結果をターミナルやログファイルで確認できます。プロジェクトを一度インポートしてから実行します。

```powershell
$godot = 'C:\Users\tomok\workspace\Godot_v4.7.2-stable_win64.exe\Godot_v4.7.2-stable_win64_console.exe'
& $godot --headless --editor --path .\simulator --import --quit
& $godot --headless --path .\simulator --fixed-fps 60 --script res://tests/terrain_test.gd
& $godot --headless --path .\simulator --script res://tests/network_test.gd
& $godot --headless --path .\simulator --script res://tests/ui_test.gd
& $godot --headless --path .\simulator --script res://tests/launch_test.gd
& $godot --headless --path .\simulator --script res://tests/smoothing_test.gd
& $godot --headless --path .\simulator --script res://tests/cub4_model_test.gd
& $godot --headless --path .\simulator --script res://tests/lidar_test.gd
& $godot --headless --path .\simulator --script res://tests/occupancy_world_test.gd
```

`occupancy_world_test.gd`を`--headless`なしで実行すると、床画像の描画も検証します。4×4画素のテスト画像を128×128画素へ拡大し、向き・表面の可視性・画素境界に補間色がないことを全画素で確認します。

Python側のワールド変換ツールの単体テストは次で実行できます。

```bash
python3 -m unittest discover -s simulator/tests -p "test_*.py" -v
```

地形の検証は坂の上り下り、実形状の階段、凹凸、壁、頭上の障害物、キャラクター同士の重なりを確認します。通信の検証はホスト＋3クライアントを別プロセスで起動し、移動同期、途中参加時の現在位置、クライアントの種類変更、ホストの主ロボット固定、クライアント切断、ホスト終了を確認します。テストが作成したプロセスは終了時に片付けます。

Cub4の検証は主ロボットだけのモデル差し替え、URDFと一致する外形・原点、頂点色の描画設定、他機の外観と既存の移動用当たり判定の維持を確認します。通信テストでは途中参加を含め全クライアントから主ロボットがCub4に見えることも確認します。`cub4_model_test.gd`の`--headless`を外すと実画面を描画し、`.godot/cub4-closeup.png`と`.godot/cub4-scene.png`に保存します。

補間の検証は、15 FPSの送信側・中継側による更新の集中とパケット欠落を再現し、30／45／60 FPSで一定速度・一定角速度の表示を確認します。従来の最新位置への追従と速度のばらつきを比較し、停止時の行き過ぎ、古い更新、通信停止・復帰、大きな位置変更も確認します。実プロセス間の通信テストでは、サーバー・クライアント双方の遠隔アバター表示と、途中参加時の時刻も確認します。

UIの検証はロビーのHost・Join・Cancelのクリック、開始・退出時の画面切り替え、TPSのマウス移動による視点操作、地図のドラッグ・回転・カーソル位置を保つズーム、全視点のスクロール／ピンチ、macOSのControl＋クリック相当の右ドラッグ、カーソル解放とフォーカス切り替え、座標表示を確認します。地図操作は高さのある地形でも検証します。入力イベントを生成するテストのため、実際のトラックパッドからイベントが届くことは別途手動で確認します。`ui_test.gd`の`--headless`を外すと、実際のウインドウでカーソルの捕捉・解放も確認します。

起動引数の検証は、エディタの共通・個別引数を組み合わせた4プロセスで行います。クライアントを先に起動してからサーバーを起動し、役割・名前・操作対象・初期視点・位置配信を確認します。引数の衝突時にロビーへ戻ることと、役割未指定時の設定も検証します。ログは`.godot/launch-test-<PID>/`、専用ポートは27346です。

通信テストの`--headless`を外すと、4つの実際のウインドウで同じ検証を行います。結果とクライアントのログは`.godot/network-test-<PID>/`へ保存します。通信テスト専用のポートは27341です。

指定のWindows版Godot 4.7.2 / Intel UHD Graphicsで、地形検証と、画面付きのホスト＋3クライアントによる通信検証を実行済みです。LAN/VPN越しの別PC接続はまだ実機検証していません。

macOS版Godot 4.7.2 / Apple M2で、UIテストをヘッドレス・画面付きの両方で実行済みです。ROS Jazzyコンテナのrosbridge 2.7.1・実C++ commanderとの往復で、W/A移動・Space停止とWASD優先・無操作時の自律復帰・フォーカスやカーソル解放に依存しない入力中継・接続断からの復帰・commanderの従来の3秒での自律指令タイムアウト停止を確認しています。コントローラーの接続切り替えと入力中継は接続状態・SDL入力を模擬して検証しています。物理コントローラーのUSB/Bluetooth接続は未検証です。スクロール／ピンチのテストはGodot入力イベントを生成して確認しており、物理トラックパッドでの操作感とLinux上の実行は未検証です。

このPCで4ウインドウを同時起動した際の各プロセスのワーキングセットは約250〜295 MB、単純合計で約1.1 GBでした。軽量な同梱ワールドを起動した状態での測定であり、将来の大きなスキャンメッシュを含む値ではありません。

## 対象範囲

Joyと速度指令のROS接続、真値オドメトリ（`/odom`）・TF・簡易LiDAR（`/scan`）・Nav2連携、および試験ワールドと2D占有地図（PGM）からのワールド生成・選択に対応しています。3Dスキャンメッシュの取り込みローダーは共通ワールド定義の拡張対象として分離しています。

全体の設計は[実装計画](./godot_simulator_plan.md)および[2D画像からGodotワールドを生成する計画](./occupancy_world_plan.md)を参照してください。
