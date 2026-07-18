# Unityシミュレータ開発方針

## 目的と対象環境

UnityでCubの差動二輪モデル、市街地環境、各種センサをシミュレートし、Docker上のROS 2プログラムの動作確認と、実機製作前のセンサ配置検討を行う。

- ROS 2: Jazzy（Docker内で実行）
- Unity: Unity 6系のバージョンをプロジェクトで固定
- Unity実行環境: Windows、Ubuntu
- 主開発環境: Windows上のUnity Editor＋WSL2上のDocker
- ROS接続: `ros2-for-unity`のJazzy対応版をstandalone構成で使用
- macOSは当面対象外とする

配布物はUnity Editorを必要としないビルド済みPlayerとし、Unity Player自体はDocker内で実行しない。

## 構成

```text
Cub_ROS/
├─ unity/
│  ├─ CubSim/                 # Unityプロジェクト
│  ├─ ros2-for-unity/         # 使用するforkまたは固定済み依存物
│  ├─ contracts/              # topic、frame、QoSの契約
│  ├─ config/                 # DDS、センサ配置等の設定
│  ├─ scenarios/              # 自動試験用シナリオ
│  └─ scripts/                # Unityの構築、ビルド、試験、起動ラッパー
├─ ros/
│  ├─ cub_simulation/         # シミュレーション共通ツール・結合試験
│  └─ mcub_bringup/           # mCubモデルとシミュレーションlaunch
├─ docker/
│  └─ ...                       # 実機と共通の通常Docker構成
└─ doc/
   └─ Unity.md
```

Unity固有の依存物、設定、シナリオ、補助ツールは原則として`unity/`以下に置く。ROSパッケージは`ros/`、Docker設定は`docker/`に置く。

`ros2-for-unity`と`ros2cs`は開発ブランチを直接追従せず、Jazzy対応を取り込んだ管理下のforkをコミットSHAまたはタグで固定する。更新時はWindowsとUbuntuの疎通試験を通してから固定先を変更する。

## 開発環境

### Windows＋WSL2

1. WindowsにUnity Hubと、プロジェクトで指定したUnity 6 Editorを導入する。
2. Windows 11のWSL2にUbuntu 22.04とDockerを導入する。ROS 2 Jazzy自体はDocker内で実行する。
3. 同一Windows PC上のUnityとWSL2を接続する場合、WSL2は`.wslconfig`で`networkingMode=NAT`を使用する。別PCのUnityと接続する場合は`networkingMode=mirrored`へ切り替える。
4. Windows FirewallとHyper-V Firewallで、Unity EditorまたはPlayerによるWSL仮想ネットワーク上のDDS UDP通信を許可する。
5. UnityとROSコンテナで、同一の`ROS_DOMAIN_ID`、RMW実装、DDS設定を使用する。

Unity用作業ツリーはWindows側、Docker用作業ツリーはWSL2のLinuxファイルシステムへ置いてもよい。その場合は両cloneを同じrevisionに保つ。`update_mcub_model.ps1`は稼働中`cub_ros`の実際のbind mount元を検出するため、WSL2ネイティブcloneを使用していてもWindows側の生成URDFを更新できる。ROSの`build`、`install`、`log`はDocker volumeまたはWSL2のLinuxファイルシステムに置き、Windows側へ生成しない。

`ros2-for-unity`のstandalone Assetを生成するWindows環境には、[ROS 2公式のWindows binary手順](https://docs.ros.org/en/jazzy/Installation/Windows-Install-Binary.html)のpixi環境、Windows版ROS 2 Jazzy、Visual Studio 2022 C++ツール、CMake、Ninjaを導入する。pixi環境へ入りROS 2 JazzyをsourceしたPowerShellから`unity/scripts/build_ros2_for_unity.ps1`を実行する。このスクリプトは長いCMakeパスを避けるため`C:\r`を一時work rootとして使い、ROS DLL、pixi側のネイティブ依存DLL、RMW選択用ament indexをstandalone Assetへまとめる。通常は`unity/scripts/setup.ps1`がAsset未生成時だけこの処理を呼び出す。

### Ubuntu

Ubuntu 24.04上にUnity 6 EditorとDockerを導入する。Unity PlayerとROSコンテナは同一の`ROS_DOMAIN_ID`、RMW実装、DDS設定で接続する。UbuntuはPlayerビルド、CI、Windows固有問題の切り分けにも使用する。

Ubuntu向けPlayerは、UbuntuホストまたはUbuntu CIランナー上で生成する。Windowsからのクロスビルドは標準フローに含めない。Ubuntuホスト上で、Jazzy対応forkからLinux用`ros2-for-unity` standalone Assetとネイティブライブラリを生成し、同じホスト上のUnity Editorをバッチモードで起動して`StandaloneLinux64` Playerをビルドする。Unity EditorとPlayerはDocker内では実行せず、結合試験に使用するROS 2だけをDocker内で実行する。

## 通信と時刻

初期開発ではUnityとROSコンテナを`rmw_fastrtps_cpp`に固定し、Fast DDSのSimple DiscoveryとUDPv4を使用する。WSL2 NATではWindowsの`vEthernet (WSL (Hyper-V firewall))`とWSL2の`eth0`が同じ仮想サブネットに属するため、このインターフェース上でmulticast discoveryとDDSデータ通信を行う。

Unity Editorは通常どおりUnity Hubから起動する。設定GUIを開くと、UnityがWindowsの稼働中IPv4インターフェースを列挙する。NAT modeではWSL名を含む仮想インターフェース、mirrored modeではdefault gatewayを持つ実LANインターフェースを優先して`DDS interface IPv4`へ設定する。シミュレーション開始時に、このアドレスだけを使用するUDPv4 Fast DDS profileを`Application.persistentDataPath/FastDDS/cubsim-udp.xml`へ生成し、Unityプロセス内へ適用する。検出結果が不適切な場合はGUIまたはPlayerの`--ros-interface-ip <IPv4>`で上書きできる。Unity Editorの起動前にROS用環境変数を設定する必要はない。

Docker側は実機開発と同じ`network_mode: host`の`cub_ros`を使い、Unity専用Compose、Discovery Server、セッションマーカー、Fast DDS用環境変数を使用しない。WSL2で通常どおり`./run.sh`を実行すればよく、UnityとDockerの起動順にも依存しない。別ホスト接続ではWSL2をmirrored modeにし、UnityのGUIで実LAN側IPv4が選ばれていることを確認する。

過去の通信構成から更新する場合は、Git管理外の`docker/home/.user_config.bash`に`FASTRTPS_DEFAULT_PROFILES_FILE`または`FASTDDS_DEFAULT_PROFILES_FILE`のexportが残っていないことを確認する。

Unityは以下を基本境界として実機相当のデータを入出力する。

- 購読: `/cmd_vel_atom`
- 発行: IMU、GNSS、LiDAR、カメラ、車輪エンコーダ相当のデータ
- 発行: `/clock`
- 発行: 評価専用の`/sim/ground_truth/odom`

ROS側の自己位置推定、オドメトリ、Nav2等を試験対象に含める。シミュレーション用launchでは実機のシリアル、RealSense等のドライバを起動せず、対象ノードに`use_sim_time:=true`を設定する。topic名、frame名、QoSは`unity/contracts/`で明文化し、UnityとROSの双方から参照できる形にする。

センサ位置とframeの基準はURDF/Xacroまたは共通設定とし、Unity Prefabだけに値を持たせない。Unity上で検討した配置をROS側のモデルへ反映でき、両者の差分を自動検査できるようにする。

## mCubテストシミュレーション

最初の動作確認対象はmCubとする。モデルの原本は`ros/mcub_bringup/urdf/mcub.urdf.xacro`とし、Unity Playerはxacroを直接解釈せず、ROS 2の`xacro`で展開した`unity/CubSim/Assets/StreamingAssets/Robots/mcub.urdf`を起動時に読み込んでロボットをスポーンする。生成URDFは直接編集しない。

Unity側ではURDFのlink、joint、box、cylinder、origin、massを読み取り、mCubの表示形状、車体Collider、左右のWheelCollider、キャスター接地点、`SLC1_link`を構成する。左右車輪は`/cmd_vel_atom`を受ける物理ベースの差動駆動とし、車輪の実回転から積算した`nav_msgs/msg/Odometry`を`/odom`へ発行する。初期のキャスター接触は自由旋回による抵抗を近似する球Colliderであり、寸法、質量、摩擦等の暫定値を実測値へ置き換えた後に段差走行を評価する。

Slamtec C1は`SLC1_link`をスキャン面としてCPU Raycastで再現し、`sensor_msgs/msg/LaserScan`を`/scan`へSensor Data QoSで発行する。初期設定はメーカー仕様に合わせ、360度、10 Hz、500点/scan、5 kHz、角度分解能0.72度、最小距離0.05 m、白色対象の最大距離12 m、黒色対象の最大距離6 m、距離分解能15 mm、測距誤差最大±30 mmとする。対象物の反射率はUnityの`LidarReflectivity`で指定し、未指定時は70%として扱う。

初期のROSインターフェースは次のとおりとする。

| 方向 | topic | message | frame |
|---|---|---|---|
| ROSからUnity | `/cmd_vel_atom` | `geometry_msgs/msg/Twist` | - |
| UnityからROS | `/odom` | `nav_msgs/msg/Odometry` | `odom` / `base_link` |
| UnityからROS | `/scan` | `sensor_msgs/msg/LaserScan` | `SLC1_link` |
| UnityからROS | `/joint_states` | `sensor_msgs/msg/JointState` | - |
| UnityからROS | `/tf` | `tf2_msgs/msg/TFMessage` | `odom` → `base_link` |
| UnityからROS | `/clock` | `rosgraph_msgs/msg/Clock` | - |

ROS接続コードは`CUB_ROS2` defineが有効なPlayerへ組み込む。WindowsではWindowsネイティブのROS 2 Jazzy環境をsourceしてJazzy対応版`ros2-for-unity`のstandalone Assetを生成し、`unity/scripts/setup.ps1`で`unity/CubSim/Assets/Ros2ForUnity`へ配置する。PlayerにはROS DLLとRMW用ament indexを同梱するため、配布先へROS 2を別途インストールする必要はない。Unity PlayerとDockerコンテナの`ROS_DOMAIN_ID`を一致させ、RMWには`rmw_fastrtps_cpp`を使用する。

### xacroからUnityモデルを更新する

人間がモデルを更新する場合は次の手順を使用する。

1. `ros/mcub_bringup/urdf/mcub.urdf.xacro`を編集する。
2. リポジトリルートから次のコマンドを実行する。

```powershell
.\unity\scripts\update_mcub_model.ps1
```

WSL2上で作業する場合は次を実行する。

```bash
./unity/scripts/update_mcub_model.sh
```

スクリプトは`docker/run_in_container.sh`を介してコンテナ内のROS 2 Jazzy版`xacro`を実行し、Unity用URDFを更新する。PowerShell版は続けてUnity Editorをバッチ起動し、必須link/joint、車輪半径、トレッド、SLC1の円筒中心とスキャン面、C1走査設定を検査する。

3. Unity Hubから`unity/CubSim`を開き、`Assets/Cub/Scenes/Bootstrap.unity`を開いてPlayする。手動検査だけを再実行する場合はUnityメニューの`CubSim > Validate generated mCub URDF`を使用する。
4. Hierarchyに生成される`mcub/base_link`以下の形状、車輪接地、キャスター接地、`SLC1_link`位置をSceneビューで確認する。モデル修正は生成されたGameObjectやURDFへ行わず、xacroへ戻して同じ手順を繰り返す。

### ビルドと疎通試験

通常のEditor開発では次の手順を使用する。

1. Unity Hubから`unity/CubSim`を開き、`Assets/Cub/Scenes/Bootstrap.unity`を開く。
2. Playを押す。表示された`CubSim Setup`でワールド、ロボット、初期X/Z/Yaw、`ROS_DOMAIN_ID`を選択する。`DDS interface IPv4`がWindowsのWSL仮想アダプターのアドレスになっていることを確認し、`Start Simulation`を押す。初期実装で選べるのは`Test World`と`mCub`である。
3. WSL2でリポジトリルートへ移動し、通常どおり`./run.sh`を実行する。UnityとDockerはどちらを先に起動してもよい。
4. 通常のラッパーから確認する。

```bash
./docker/run_in_container.sh ros2 topic list
./docker/run_in_container.sh ros2 run cub_simulation cub_simulation_smoke
```

`topic list`では`/clock`、`/cmd_vel_atom`、`/joint_states`、`/odom`、`/scan`、`/tf`を確認できる。Unity Editorは非アクティブでもシミュレーションとROS通信を継続する。`cub_simulation_smoke`はLiDAR、odometry、joint states、TFを受信し、`/cmd_vel_atom`へ前進指令を送ってUnity上の移動まで検査する。Unityは実機のhardware launchが提供するセンサ・駆動境界を置き換えるため、ROS側では実機の`control_mcub`、`wheel_odometry`、`sllidar_ros2`を同時に起動しない。mCub向けパッケージをクリーンビルドするときは、リポジトリルートの`target.env`を`CUB_TARGET=mcub`（全機種を対象にする場合は`all`）にしてからビルドする。RVizを使用するときは、xacroから`/robot_description`と`/tf_static`を生成する`robot_state_publisher`を次のコマンドで起動する。

```bash
./docker/run_in_container.sh ros2 launch mcub_bringup simulation.launch.py
```

手動で走行指令を送る場合は、Unity側の安全停止タイムアウトを超えないように連続送信する。

```bash
./docker/run_in_container.sh "ros2 topic pub -r 10 /cmd_vel_atom geometry_msgs/msg/Twist '{linear: {x: 0.1}, angular: {z: 0.0}}'"
```

```powershell
# xacro展開、ros2-for-unity standalone Asset配置、CUB_ROS2有効化
.\unity\scripts\setup.ps1

# Unityのモデル契約とC#コンパイルを検査
.\unity\scripts\test.ps1

# Windows Playerを生成
.\unity\scripts\build.ps1

# 通常のcub_rosコンテナとPlayerで、前進指令、/odom、/scanを自動検証
.\unity\scripts\e2e.ps1 -RosDomainId 0
```

`e2e.ps1`は配布Player向けの自動回帰試験であり、通常のEditor開発には必須ではない。WSL2上の通常の`cub_ros`を必要なら起動し、`cub_simulation`をビルドしてからPlayerを開始する。最初に`ros2 topic list`でUnityのtopicを検査し、続いて`cub_simulation_smoke`が`/cmd_vel_atom`へ前進指令を送り、`/odom`の移動量、`/joint_states`、`/tf`、および`/scan`のframe、500点、周期、角度分解能、距離範囲を検査する。WSL2側のリポジトリ位置が異なる場合は`-WslRepoRoot`、自動検出したNICを上書きする場合は`-RosInterfaceIp`を指定する。

## ワールドとロボットの選択

Unityプロジェクトと配布Playerは機体やワールドごとに分割せず、Windows用とUbuntu用にそれぞれ1種類とする。Playerは共通のBootstrap Sceneを最初にロードし、選択されたワールドSceneをAdditive Loadした後、選択されたロボットPrefabをスポーンする。

Bootstrap Sceneには`SimulationSessionManager`、`WorldLoader`、`RobotSpawner`、ROS接続、シミュレーション時計、設定GUIを置き、Player終了まで維持する。ワールドSceneには地形、建物、Collider、Lighting、Physics Material、SpawnPoint等のワールド固有要素だけを置き、ロボットやROSノードは直接配置しない。

ワールドは次の構成で管理する。

```text
unity/CubSim/Assets/Cub/Worlds/
├─ UrbanBasic/
│  ├─ UrbanBasic.unity
│  ├─ UrbanBasicDefinition.asset
│  └─ Preview.png
├─ UrbanSteps/
│  ├─ UrbanSteps.unity
│  ├─ UrbanStepsDefinition.asset
│  └─ Preview.png
└─ MobilityLab/
   ├─ MobilityLab.unity
   ├─ MobilityLabDefinition.asset
   └─ Preview.png
```

各ワールドには`WorldDefinition` ScriptableObjectを用意し、安定したworld ID、表示名、Scene、説明、プレビュー画像、SpawnPoint、対応機体、対応シナリオ、GNSS原点、物理設定、ROS地図ID、タグを保持する。world IDはUnity、ROS、コマンドラインで共通とする。ROSが使用する地図、ウェイポイント、経路等は`ros/cub_simulation/worlds/<world-id>/`に置き、CIでUnity側のworld IDとの対応を検査する。

初期実装ではワールドSceneをPlayerへ含める。ワールド数や容量が増え、個別配布が必要になった場合は、同じ`WorldDefinition`とworld IDを維持したままAddressablesによる別配布へ移行する。

### 設定GUI

引数なしでPlayerを起動した場合は、シミュレーション開始前に設定GUIを表示する。GUIではワールド、ロボット、SpawnPoint、シナリオ、ROS/DDS設定を選択できるようにする。ワールドが対応していない機体や、必要なセンサを持たないシナリオは選択不可とし、理由を表示する。

開始後のGUIでは、一時停止、再開、初期位置へのリセット、SpawnPointの変更、センサ表示、ROS接続状態の確認、セッション終了を行えるようにする。ワールドまたは機体を変更する場合は現在のセッションを終了し、設定GUIへ戻って新しいセッションを開始する。

新しいセッションは次の順序で構築する。

1. 物理演算とシミュレーション時計を停止する。
2. ROS publisher、subscriber、nodeを停止する。
3. ロボットを破棄する。
4. 現在のワールドSceneをUnloadする。
5. 新しいワールドSceneをAdditive Loadする。
6. 指定されたSpawnPointへロボットを生成する。
7. ROS接続とシミュレーション時計を初期化する。
8. 物理演算とシミュレーションを開始する。

同じワールドと機体のまま初期位置へ戻す軽量リセットでは、ROSノードを作り直さず、シミュレーション時刻を単調増加させる。ワールド、機体またはシナリオを変更する場合は新規セッションとして扱い、必要に応じてROS launchも再起動する。これにより古いTF、Nav2状態、時刻巻き戻りの影響を避ける。

### コマンドラインと自動実行

GUIとコマンドラインは同じ設定モデルを使用する。コマンドライン引数はGUIの初期値として扱い、`--autostart`が指定された場合だけGUIを省略して開始する。

```text
CubSim.exe --world urban_steps --robot cub3
CubSim.exe --world mobility_lab --robot spidar --spawn curb_left
CubSim.exe --world mobility_lab --robot cub3 --scenario curb_50mm --autostart --headless
```

引数の優先順位は、コマンドライン、保存済みユーザー設定、プロジェクト既定値の順とする。手動利用時の最終選択はUnityの`persistentDataPath`以下に保存する。自動試験ではworld ID、robot ID、SpawnPoint、scenario、乱数seedを明示し、ワールドまたは機体の変更ごとに新しいセッションを開始する。

`unity/scripts/run.*`と`unity/scripts/e2e.*`はGUIと同じworld ID、robot ID、scenarioを受け取り、ROS側へも対応する`CUB_TARGET`とworld IDを渡す。Unity側とROS側で機体やワールドを個別に選択させない。

高低差の評価には大規模な市街地ワールドに加えて、小規模な`MobilityLab`を用意する。50 mm段差への正面進入、片輪進入、段差からの降下、横断勾配、底面接触、空転等をSpawnPointとscenarioの組合せとして管理し、自動回帰試験に使用する。

## センサシミュレーション

各センサはロボットPrefabへ複数配置できる独立したコンポーネントとして実装し、センサ固有仕様、計測バックエンド、ROS出力を分離する。センサの位置と向きはURDF/Xacroのframeを基準とし、Unity側だけに外部パラメータを保持しない。

```text
Sensor Component
├─ Sensor Definition       # 画角、走査パターン、距離、ノイズ等
├─ Measurement Backend     # CPU/GPUによる計測
├─ Buffer / Scheduler      # 実行時刻と非同期バッファ
└─ ROS Publisher           # message、topic、frame、QoS
```

全センサは共通の`SensorScheduler`へ登録する。センサごとに独立した周期と位相を設定し、LiDAR、カメラ、点群生成を同一frameへ集中させない。計測timestampにはシミュレーション時刻を使用し、ROS送信が追いつかない場合は古い計測を蓄積せず破棄する。

### RGB-Dカメラ

使用予定のステレオカメラは実機ドライバがDepthImageと色付き点群を出力するため、Unity内でステレオマッチングは実行しない。実機の推定結果を模擬するRGB-Dカメラとして、同一仮想視点からRGB画像とlinear depthを描画し、同じtimestampのDepthImageと色付きPointCloud2を生成する。

```text
RGB RenderTexture ─┐
                    ├─ Compute Shader ─┬─ DepthImage
Depth RenderTexture ┘                  └─ XYZ＋RGB PointCloud2
```

センサ定義には解像度、フレームレート、内部パラメータ、FOV、最小・最大距離、距離量子化、距離依存ノイズ、エッジや反射率による欠損、optical frameを保持する。左右画像やステレオ処理自体を試験対象に追加するまでは、左右2台分の描画は行わない。

色付き点群はデータ量が大きいため、DepthImage、RGB画像、色付き点群の出力周期を個別に設定可能にする。点群には画素間引き、最大距離、無効点除去を設定できるようにし、標準設定ではDepthImageより低いレートで発行する。GPU上で座標と色を連続バッファへ格納し、点ごとのC#オブジェクトは生成しない。

### LiDAR

初期対応対象はVelodyne VLP-32C、Livox Mid-360、HESAI JT128とする。各センサの照射方向、発射順序、ring/channel、時刻、距離、return mode、ノイズ、PointCloud2 field構成を`LidarSensorDefinition`として保持する。

```text
LidarSensor
├─ Vlp32Definition
├─ Mid360Definition
└─ Jt128Definition
```

VLP-32Cは32本の垂直角と回転走査、Mid-360は非反復走査、JT128は128 channelと広い垂直FOVを個別に再現する。Mid-360等の複雑な走査は、実機または公式SDKから取得した「計測時刻、方位角、仰角」の走査テンプレートを利用できるようにする。

初期実装はSingle/First Returnとし、Dual Return、半透明物体、ガラス、雨、霧、レーザー干渉等は追加機能として扱う。intensityはマテリアル反射率、入射角、距離を基に生成し、実機計測との比較によってノイズと欠損モデルを調整する。

VLP-32CとMid-360を同時搭載する構成を標準的に扱えるよう、`LidarSensor`とGPU/CPUバッファは複数インスタンスを前提とする。両センサは位置と走査時刻が異なるため、それぞれの位置から計測し、Depthや計測結果を安易に共有しない。topic、frame ID、QoS、scan phaseもセンサごとに独立させる。

### 計測バックエンド

LiDARは次のバックエンドを共通インターフェースで切り替えられるようにする。

```text
ILidarBackend
├─ GpuDepthBackend       # 標準バックエンド
├─ CpuRaycastBackend     # CI、低負荷、比較検証
├─ DxrBackend            # Windows用の将来オプション
└─ RglBackend            # NVIDIA用の将来オプション
```

標準の`GpuDepthBackend`はLiDAR位置から周囲のDepthを描画し、Compute Shaderでセンサ固有の照射方向をサンプリングする。CUDAや特定ベンダーのRT coreを必須とせず、WindowsではDirectX、UbuntuではVulkan、将来macOSへ対応する場合はMetalを使用する。これによりWindowsとUbuntuのNVIDIA/AMD GPUで同じ基本実装を利用する。

GPUからの結果取得には`AsyncGPUReadback`と二重または三重バッファを使用する。Depth描画、点群生成、GPU readback、PointCloud2構築、ROS publishを直列に待たず、前回計測を送信している間に次の計測を生成する。

`CpuRaycastBackend`は`RaycastCommand.ScheduleBatch`でRaycastを分割・並列実行する。GPUを使用しないCI、点数を間引いた試験、GPU結果との比較に使用する。VLP-32CとMid-360の実点数を含むCPU実行も評価するが、複数LiDAR、RGB-D、車体物理を同時実行する標準構成にはGPUを使用する。

Robotec GPU Lidar等のCUDA/OptiX実装はWindows/LinuxのNVIDIA GPU用オプションとして利用可能にするが、標準機能を依存させない。AMD GPUや将来のmacOSでもセンサ構成とROS出力が変わらないよう、バックエンドの上位にセンサ定義とpublisherを置く。

### 走査時刻と動体

LiDARの1 scanを同一時刻の一括計測として扱わず、実機の発射順序に従う複数ブロックへ分割できるようにする。高精度モードではセンサや動体の移動を各ブロック時刻へ反映し、回転走査および非反復走査の歪みを再現する。高速モードではscan開始時点の姿勢を使用して計算量を減らす。

RGB、DepthImage、色付き点群は同一計測について同じtimestampを使用する。LiDAR間およびカメラとの計測位相は実機同様に独立させ、必要な場合だけROS側で同期する。

### ROS出力

初期実装では実機ドライバの後段と同じROS messageとtopicを出力し、実機の生UDP packet生成は対象外とする。想定する出力は次のとおり。

```text
/velodyne_points
/livox/lidar
/camera/depth/image_raw
/camera/depth/camera_info
/camera/color/image_raw
/camera/depth/color/points
```

実際のtopic名は機体プロファイルで指定する。PointCloud2は`x`、`y`、`z`、`intensity`、`ring`、点ごとの時刻、必要なセンサ固有fieldを実機ドライバの出力へ合わせる。QoSはSensor Data用途を基本とし、送信キューを小さく保つ。

PointCloud2は連続した再利用可能バッファへ直接構築し、メモリプールを使用する。特に色付き点群はLiDARよりデータ量が大きくなるため、GPU計測時間だけでなく、GPU readback、C#バッファ構築、DDS serialization、ネットワーク送信を個別に計測する。

### 品質プリセット

センサ品質は用途に応じて切り替えられるようにする。

| プリセット | 用途 | 主な設定 |
|---|---|---|
| `CI` | 自動試験 | CPUまたは低解像度GPU、LiDAR間引き、色付き点群無効または低レート |
| `Realtime` | 標準利用 | GPU、LiDAR実点数、Single/First Return、DepthImage標準レート、色付き点群間引き |
| `Fidelity` | センサ評価 | 走査時刻分割、intensity、ノイズ、走査歪み、色付き点群高解像度 |
| `Vendor` | 対応GPUでの高性能実行 | DXR、RGL等のオプションバックエンド |

将来M2 Macへ対応する場合はMetalによる`GpuDepthBackend`を使用し、点数、LiDAR更新レート、Depth解像度、色付き点群レートを調整可能にする。Mid-360、VLP-32C、JT128、RGB-Dを個別および同時にベンチマークし、品質プリセットごとに処理遅延とdrop数の上限を定める。

### 検証

センサ実装は静的な校正用ワールドと実機rosbagを使用して検証する。照射方向、point数、timestamp、frame、PointCloud2 field、距離誤差、ノイズ分布を比較する。性能試験では複数LiDAR、RGB-D、200 Hzの車体物理、市街地ワールドを同時実行し、次を個別に計測する。

- Depth/Raycast生成時間
- Compute Shader実行時間
- GPU readback遅延
- PointCloud2構築時間とメモリ割り当て
- ROS serializationと送信帯域
- 計測drop数
- physics fixed stepの遅延

## 標準ラッパースクリプト

Unity EditorのGUI操作を必須にせず、開発者とCodexが同じ手順を実行できるよう、実装時に次のラッパーを`unity/scripts/`へ用意する。

| Windows | Ubuntu | 用途 |
|---|---|---|
| `setup.ps1` | `setup.sh` | 固定済み依存物の取得・Unity Assetの配置・環境検査 |
| `build.ps1` | `build.sh` | Unity Playerのバッチビルド |
| `test.ps1` | `test.sh` | Unity EditMode/PlayModeテスト |
| `run.ps1` | `run.sh` | ビルド済みPlayerの起動 |
| `e2e.ps1` | `e2e.sh` | ROSコンテナとPlayerを使った疎通・シナリオ試験 |

Windowsでの標準的な使用例を示す。

```powershell
# 初回セットアップと環境検査
.\unity\scripts\setup.ps1

# Windows Playerをビルド
.\unity\scripts\build.ps1 -Target Windows

# Unity単体テスト
.\unity\scripts\test.ps1

# ビルド済みPlayerを起動
.\unity\scripts\run.ps1 -Scenario urban-basic

# ROS 2を含む結合試験
.\unity\scripts\e2e.ps1 -Scenario urban-basic
```

Ubuntuでは対応する`.sh`を使用する。ROSコマンドはラッパー内部から`docker/run_in_container.sh`を介して実行する。各スクリプトは非対話実行、明確な終了コード、ログ出力先指定に対応させる。Playerにはシナリオ、DDS設定、乱数seed、固定時間刻み、headless実行等をコマンドラインで渡せるようにする。

Ubuntu向けPlayerの標準的な生成・検証手順を示す。

```bash
# Linux用ros2-for-unity standalone Assetの生成と環境検査
./unity/scripts/setup.sh

# Ubuntu PlayerをUbuntuホスト上でビルド
./unity/scripts/build.sh --target Linux

# Unity単体テスト
./unity/scripts/test.sh

# ROS 2コンテナを含む結合試験
./unity/scripts/e2e.sh --scenario urban-basic
```

安定後はこの一連の処理をUnityライセンスが利用可能なUbuntu CIランナーで実行し、ビルド済みPlayerを配布用artifactとして保存する。

## 開発・試験フロー

1. `setup`で依存関係と環境を検査する。
2. UnityまたはROSコードを変更する。
3. Unity単体テストと対象ROSパッケージのビルドを行う。
4. `e2e`でPlayerとROSコンテナを起動し、topic、TF、時刻、走行結果を検証する。
5. 必要に応じてrosbag、JUnit結果、Playerログ、スクリーンショットを保存する。
6. WindowsとUbuntuのビルド・試験成功後に配布用Playerを生成する。

UnityはAsset SerializationをForce Text、Version ControlをVisible Meta Filesに設定する。`Library/`、`Temp/`、`Logs/`、`Builds/`等の生成物はGit管理しない。大型の都市モデル、テクスチャ、メッシュはGit LFSで管理する。SceneやPrefabへの手作業を減らし、ロボット、センサ、シナリオの構成は可能な限りC#とレビュー可能な設定ファイルで表現する。
