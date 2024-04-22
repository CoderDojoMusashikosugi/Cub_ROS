# Cub_ROS

## ファイル構造
- docker
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

## このレポジトリをROSのパッケージ置き場として使う
- これを colcon_ws/src/以下にクローンする。
- colcon_ws/src/Cub_ROS/ros/cub_bringup/package.xml のようなファイルパスでパッケージが配置される。
- colcon_wsに移動して、そのままビルドすると使える。

## Docker環境を使う
- 対応環境
  - CPU: arm64
  - OS : Linux系(Ubuntu22.04で動作を確認)

- 今後対応したいもの
  - CPU: Intel
  - OS : Windows, macOS, RaspberryPi OS(今でも動くとは思う), Jetpack(これも動くとは思う)
  - GPU: NVIDIA, AMD, Intel
  - dockerイメージの配布

### Docker環境を構築する
```
./scripts/docker_install.sh
sudo reboot
```

### Docker環境をビルドする
```
./scripts/docker_build.sh
```

立ち上げ中のコンテナが終了するので注意

### Docker環境を起動する
```
./run.sh
```

プロンプトがcub@dockerになる。  
この環境内ではros2コマンド等が使え、rvizなども起動出来る。  
ホームディレクトリに置いたファイルはdocker/home以下に出てくる。  
この環境から抜けるには、exitコマンドを実行する。

### Dockerコンテナを停止する
```
./stop.sh
```

### 一番新しいやつ以外のDockerイメージを削除する
ストレージが一杯になったとき用
```
./scripts/docker_container_remove.sh
```
