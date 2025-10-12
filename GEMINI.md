# Gemini Project Context: Cub_ROS

## プロジェクト概要
ROS2 Jazzyを利用した自律移動ロボットCubの制御ソフトウェア。

センサ情報をDockerコンテナ上のROS2ノードで処理し制御コマンドを生成、シリアル通信経由でM5 Atomに渡して車体を制御する。

## 主要なコマンド
- ROS2のビルド(コンテナ内で実行)
    - `cb`
- ROS2で単体のパッケージをビルド(コンテナ内で実行)
    - `cbs <package name>`

## コンテナ内でのコマンド実行
コンテナ内でのコマンド実行は、ホストから `docker/run_in_container.sh` スクリプトを利用して行います。  
`ros2`コマンドは、この機能を利用してコンテナ中で実行する必要があります。  
このスクリプトは、コンテナが停止している場合に自動で起動し、指定されたコマンドを実行します。`.bashrc`で定義されたエイリアス (`cb`, `cbs`など) も利用可能です。  

コンテナ内のホームディレクトリ`/home/cub/`は、ホストの`./docker/home/`にボリュームマウントされています。  
コンテナ内の`/home/cub/colcon_ws/src/cub/`は、ホストの`./ros/`にボリュームマウントされています。

### 使い方
```bash
./docker/run_in_container.sh <コンテナ内で実行したいコマンド>
```

### 例
```bash
# パッケージを全てビルドする
./docker/run_in_container.sh cb

# パッケージを一つビルドする(この場合はcub_bringupを選択)
./docker/run_in_container.sh cbs cub_bringup

# コンテナ内のファイルをリストする
./docker/run_in_container.sh ls -l
```

## 技術スタック
- 言語: C++, Python
- フレームワーク: ROS2 Jazzy
- ビルド・実行環境: Docker

## ディレクトリ構成
- `ros/`: ROS2パッケージの配置場所、コンテナ内では`~/colcon_ws/src/cub/`にマウントされている。
- `docker/`: Docker関係のファイルが配置されている場所
- `m5atom/`: M5 Atomマイコン向けのPlatformIOプロジェクトの配置場所
- `scripts/`: 主にホストで実行するセットアップスクリプトを保管している

## 既存のドキュメント
- `./README.md`: このプロジェクトの利用方法について詳しく記載している
- `./docker/README.md`: Docker環境の構成や利用方法について詳しく記載している