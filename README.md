# Cub_ROS

## はじめに
このレポジトリでは、つくばチャレンジのCoderDojo武蔵小杉チームが開発するCub3とmCub向けのソフトウェアを管理している。

## 使い方
### 開発環境の用意
- Cub3またはmCubを用意
  - UbuntuやWSLgやmacOS等、[ここのサポート環境](./docker/README.md)の項目にあるパソコンでも利用可能
- このレポジトリをClone
  - `cd ~` (gitレポジトリをホームディレクトリ直下へ配置することを前提に解説するが、他のディレクトリでも問題ない)
  - `git clone https://github.com/CoderDojoMusashikosugi/Cub_ROS.git`
  - `cd ~/Cub_ROS` (以降はこのディレクトリ内で実行することを前提に解説する)
  - `git submodule update --init`
- 実行に必要なソフトウェアをインストール
  - Dockerをインストール
    - `./docker/install.sh`
    - 環境によってはインストール出来ない。その場合は自力で。
    - 再起動の指示が出たら、再起動して`cd ~/Cub_ROS`に自力で帰り、次の手順を続ける。この項目の最後の再起動でまとめてやっても大丈夫。
  - **Cub3の場合** Cub3向けデバイス設定をインストール
    - `./scripts/install_host_settings.sh`
  - **mCubの場合** mCub向けデバイス設定をインストール
    - `./scripts/mcub_host_settings.sh`
- 設定
  - **Cub3の場合** target.envを`CUB_TARGET=cub3`にする (デフォルトでそうなっているので基本的には対応不要)
  - **mCubの場合** target.envを`CUB_TARGET=mcub`にする。**ここ変更しないとmCubで立ち上がらないので注意**。
  - 各ロボットの設定は`./docker/environment/`内の設定ファイルで管理される。
- 再起動する
  - 再起動後`cd ~/Cub_ROS`に返ってくるのをお忘れなく。

### M5 Atomをセットアップする
- [m5atom/README.md](m5atom/README.md)の内容の通りにM5 Atomにプログラムを書き込む。

### 開発・実行環境に入る
- Cub3やmCubをGUI操作可能な画面を用意し、そこでターミナルを立ち上げて`./run.sh`を実行
  - 言い換えれば、コンピュータ起動後最初の./run.shはssh経由でやらないほうが良い。やるとGUIにRViz等が出なくなってしまう。もしsshから初回起動してしまった場合は、一旦`exit`して`./stop.sh`の後再度`./run.sh`すべし。
    - こうなるのは、dockerコンテナを立ち上げた際の画面にGUIを出す設定となっているため。一度stopすると良いというのは、dockerコンテナを立ち下げる操作であるため。
    - MacはVNC環境側に画面を出すため、どこで起動しても大丈夫。
  - 初回は10GB程度のダウンロードが入る。完了までひたすら待つ。
- 完了したらDocker環境に入れる。
  - もしこのシェルを`exit`しても、コンテナ自体は立ち上がり続けている。再度`./run.sh`すれば再度入れる。
- 他のターミナルを開いて同様に./run.shすればもっとシェルを増やせる。
  - GUIのアプリをssh経由で立ち上げようとも、画面はCub3/mCub側に出る。
- 以降はこの./run.shまで実行されていることを前提に解説する。
- RViz2(`rviz2`で起動する)等のGUIは基本的にはデスクトップ上に出るが、macでだけ出せないので代わりにwebブラウザから http://localhost:6080 にアクセスした先に出す。詳しくは[Tiryoh/docker-ros-desktop-vnc](https://github.com/Tiryoh/docker-ros-desktop-vnc)を参照。

### ROSパッケージをビルドする
- ビルドを実行する
  - `cd colcon_ws`
  - `colcon build --symlink-install`
  - (↑を入力するのが長くて面倒なので、一応`cb`というエイリアスで上記を一気に実行して元のディレクトリに戻れるようにしてある。)
- もし`colcon build --symlink-install`がエラー終了する場合は↓
  - 依存するライブラリが不足していそうな場合は、一度./run.shの環境から`exit`して、`./stop.sh`してから再度`./run.sh`するとビルドが通るようになっている場合がある。(コンテナ立ち上げ直し)
  - メモリ不足で止まっていそうな場合は、`MAKEFLAGS="-j 1" colcon build --symlink-install`のようにして1並列でビルドさせることでメモリ使用量を削減可能。
    - ここで、1の部分が並列数なので2や3にしていくとメモリ使用量が増えてビルドは早くなる。
- もしパッケージ単体でビルドしたければ(cub_bringupを例に)
  - `colcon build --symlink-install --packages-select cub_bringup`
    - ここで、cub_bringupの部分が選択するパッケージなので、ここをお好みのパッケージ名にする。
    - (↑を入力するのが長くて面倒なので、`cbs cub_bringup`で同じ操作が可能にしてある。)
- `source ~/.bashrc`を実行してビルド結果をbashから呼び出せるようにする。
  - (↑を入力するのが長くて面倒なので、`bashrc`で同じ操作が可能にしてある。)
  - これはcolcon buildごとに実行する必要は必ずしも無くて、パッケージ追加時やパッケージへのファイル追加時に実行すると吉。

### ロボットを起動する
- センサやアクチュエータを`ros2 launch cub_bringup launch_at_boot.launch.py`で立ち上げる
  - これはロボットの起動と共に自動で起動させるはずのもの。起動自体の設定をしていない場合はこの通りに手動で実行する。
  - ロボット自体の電源を落とすまで開きっぱなしがおすすめ。
- Cub3/mCub向けのROS2ノードを実行する
  - `ros2 launch cub_bringup common.launch.py`
- ここまで実行すれば、DualSenseで操作が可能になっている
  - L2(今はL1も同様の機能)を押しながら左スティック前後左右で、前後移動と左右回転ができる。
  - DualSenseの矢印ボタン上下で0.1m/sずつ最高速度の変更ができる。デフォルトでは1.2m/s。
  - L2は自律走行時にも重要なコマンド。自律走行のコマンドを手動操縦で上書きできるため、危険な場合の操作オーバーライドに使える。ただしL2離せば元通りなのに注意。
  - DualSenseが無い場合の手動操縦には、`ros2 run teleop_twist_keyboard teleop_twist_keyboard cmd_vel:=cmd_vel_atom`等で`/cmd_vel_atom`にデータを流すと動く。

### 地図を作成する(2D)
- 機体を手動で初期位置に持って行く
  - 理由は後述するが、地図作成開始時にオドメトリが原点のままである場合、その後の作業が楽で大変おすすめ。
- 地図作成用ノードを立ち上げ
  - `ros2 launch cub_bringup mapping.launch.py`
- 正しく起動できていれば、RVizに地図が出来上がっていく。手動操縦して生成される地図の範囲を広げていこう。
- 最後に地図を保存する
  - `mkdir ~/maps/map1 && ros2 run nav2_map_server map_saver_cli -f ~/maps/map1/map --ros-args -p save_map_timeout:=1000.0`
  - これで`~/maps/map1`ディレクトリに地図が保存される。
- 地図データを確認する
  - 画像データなので普通に開ける。
- 全て完了したら、mapping.launch.pyを落とす
  - launch_at_boot.launch.pyは立ち上げっぱなしで大丈夫。

### 作成した地図上で自律走行する(2D)
- ロボットを地図作成開始時の初期位置に持っていく
- ロボットを起動、地図のディレクトリ指定を添えて
  - `ros2 launch cub_bringup navigation.launch.py map:=/home/cub/maps/map1/map.yaml`
- ↑の操作で新しく立ち上がったRVizの画面に初期姿勢を指定する
  - [こちらの記事の「Navigation2の実行」の項目](https://qiita.com/porizou1/items/d63a41fc1e478dfa5ab6#navigation2%E3%81%AE%E5%AE%9F%E8%A1%8C)を参考に、初期姿勢に2D Pose Estimateの矢印を置く。この初期姿勢は、今までの手順を守れば地図の原点で向きは赤い棒の方向に設定すれば良くなって便利。
- ゴールを設定する
  - 上記記事の通りにNavigation2 Goalを設定すると、そこに向けてロボットが走っていくはず。
- 走らせ飽きたら、navigation.launch.pyを落とす
  - launch_at_boot.launch.pyは立ち上げっぱなしで大丈夫。

### その他便利機能
- ストレージが一杯の時は
  - Dockerイメージがストレージを圧迫している場合は、`./docker/remove.sh`で最新の以外を削除できる。
- コーディングエージェントを使うには
  - 全体的には、コード生成はそのままやってくれる。コンテナ内でしか使えない機能をAIに操作してもらうために./docker/run_in_container.shを用意している。
  - gemini-cli向けにはGEMINI.mdが配置されているので、これを読んで操作してもらう。
  - github-copilot向けには.github/copilot-instructions.mdがある。実はGEMINI.mdへのシンボリックリンク。
  - claude code向けにはCLAUDE.mdがある。実はGEMINI.mdへのシンボリックリンク。
  - codex向けにはAGENTS.mdがある。実はGEMINI.mdへのシンボリックリンク。


## Dockerの技術情報
[dockerディレクトリ内のREADME.md](./docker/README.md)を参照
