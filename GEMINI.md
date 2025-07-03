# Gemini Project Context: Cub_ROS

## プロジェクト概要
ROS2 Humbleを利用した自律移動ロボットCubの制御ソフトウェア。

センサ情報をDockerコンテナ上のROS2ノードで処理し制御コマンドを生成、シリアル通信経由でM5 Atomに渡して車体を制御する。

## 主要なコマンド
- Dockerコンテナを起動しコンテナに入る
    - `./docker/run.sh`
- ROS2のビルド(コンテナ内で実行)
    - `cb`
- ROS2で単体のパッケージをビルド(コンテナ内で実行)
    - `cbs <package name>`

## 技術スタック
- 言語: C++, Python
- フレームワーク: ROS2 Humble
- ビルド・実行環境: Docker

## ディレクトリ構成
- `ros/`: ROS2パッケージの配置場所、コンテナ内では`~/colcon_ws/src/cub/`にマウントされている。
- `docker/`: Docker関係のファイルが配置されている場所
- `m5atom/`: M5 Atomマイコン向けのPlatformIOプロジェクトの配置場所
- `scripts/`: 主にホストで実行するセットアップスクリプトを保管している

## 既存のドキュメント
- `./README.md`: このプロジェクトの利用方法について詳しく記載している
- `./docker/README.md`: Docker環境の構成や利用方法について詳しく記載している
