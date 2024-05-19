# Cub_ROS

## ファイル構造
- docker: Docker関連のファイルを置いている。
  - home: この中に置いたファイルはコンテナの中からも見れる
    - .user_config.bash: ./run.shを実行したら最初だけ生成されるファイルでその後は消されないので、個人用の設定を入れておく。ROSのPC間通信設定などに。
  - additional_pkgs.bash: dockerイメージにインストールしたいパッケージを書くなどに使う。docker_build.shでイメージ生成時に実行される。
  - docker-compose.yml: デバイスとの接続設定などに使う
- ros: ここにrosのパッケージを置く。以下は配置例。
  - cub_bringup
    - package.xml
- scripts: スクリプト置き場、主にdocker用だが、関係ないのを置いても良い
- README.md: これ。
- run.sh & stop.sh: docker用、後述する。

## Docker環境を使うには
- 対応環境
  - CPU: arm64, x86_64
  - OS
    - Ubuntu (22.04で動作確認)
    - RaspberryPi OS 64bit (RaspberryPi5で動作確認)
    - macOS (macOS Monterey な M2 Macbook Airで動作確認)
    - Ubuntu on WSLg (Windows11で動作確認)
  - GPU
    - GPUアクセラレーションは何も有効化していない

- 今後対応したいもの
  - OS : Jetpack(これも動くとは思う)
  - GPU: NVIDIA dGPU, NVIDIA Jetson, AMD, Intel
  - 何らかのシミュレーター対応

### このレポジトリをcloneする
```
git clone https://github.com/CoderDojoMusashikosugi/Cub_ROS.git -b feature-add-docker-env
cd Cub_ROS
```

### Dockerをインストールする
```
./docker/install.sh
sudo reboot
```

Raspberry Pi OS と Ubuntu に対応している。  
Windows(WSL)やmacOSの場合はDocker Desktopを手動でインストール。

### Docker環境を起動する
```
./run.sh
```

基本的には、環境に対応するdockerイメージをpullして起動する(3分くらい)。なければbuildしてから起動(15分から180分)。  
起動すると、プロンプトがcub@dockerになる。この環境内ではros2コマンド等が使える。  
RVizは `ros2 launch cub_visualization rviz.launch.py` で起動する。基本的には画面にそのまま出るが、出せないmacOSでだけ http://localhost:6080 に出る。  
ホームディレクトリに置いたファイルはdocker/home以下に出てくるので、消えてほしくないファイル(ログとか)はここに置く。逆に、ここ以外に置いたファイルは./stop.shで消える。  
この環境から抜けるには、exitコマンドを実行する。exitしてもコンテナ自体は終了しない。

初回の./run.sh実行で `docker/home/.user_config.bash` が生成される。これはgit commitされないので、ここに環境固有の設定を追記する。  
特に、外部との通信設定の有効化/無効化はここで設定する。

### ROS2パッケージをビルドする
`./run.sh`を実行した状態で

```
cd colcon_ws/
colcon build --symlink-install
```

### Dockerコンテナを停止する
```
./stop.sh
```

### Docker環境をビルドする
```
./docker/build.sh
```

立ち上げ中のコンテナが終了するので注意

追加したいパッケージは、基本的にはROS2パッケージの`package.xml`に`<depend>`タグで追記することでrosdep経由でインストールする。  
それで追加出来ないものは`docker/additional_pkgs.bash`にインストールの司令を書く。

### 一番新しいやつ以外のDockerイメージを削除する
ストレージが一杯になったとき用
```
./docker/remove.sh
```

## このレポジトリをROSのパッケージ置き場として使うには
- これを colcon_ws/src/以下にクローンする。
- colcon_ws/src/Cub_ROS/ros/cub_bringup/package.xml のようなファイルパスでパッケージが配置される。
- colcon_wsに移動して、そのままビルドすると使える。
