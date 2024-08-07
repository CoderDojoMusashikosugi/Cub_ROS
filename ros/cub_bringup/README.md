# cub_bringup
Cub2/mCubの起動用launchファイルと、その他便利ツールの置き場

`cub_bringup cub.launch.py` とか実行したらCub2の全てのプロセスやデバッグツールが起動して欲しいなと思っている。

## turtlebot3 関連ファイルとその使い方
Cub2とは何の関係ないファイル。  
turtlebot3_から始まるlaunchファイルを使うことで、turtlebot3のシミュレーション環境を起動し自律走行のテストが出来る。

### Docker環境の構築と起動
- clone
    - `git clone https://github.com/CoderDojoMusashikosugi/Cub_ROS.git`
- [レポジトリのREADME.md](../../README.md)の通りにDockerをインストールする
- gitのsubmoduleを更新(全ての参照先のgitレポジトリからデータを取得)
    - `cd Cub_ROS`
    - `git submodule update --init --recursive`
- Docker環境を起動(環境がなければビルドやダウンロード)
    - `./run.sh`
    - 初回はすごい時間かかる
    - 実行完了後は、プロンプトが`cub@docker:~$ `になっていればDocker環境に入れたということ。
- Docker環境内でROS2パッケージをビルドする
    - `cd colcon_ws`
    - `MAKEFLAGS="-j 1" colcon build --symlink-install`
        - `MAKEFLAGS="-j 1"`の部分はビルドにメモリを食いまくるせいで失敗する場合のため並列ビルドを無効化する命令。メモリが16GBしかない貧弱な環境では付けておくと良い。メモリが64GB程あれば通常の`colcon build --symlink-install`でOK。
        - 初回すごい時間がかかる、特に"-j 1"つけた場合は。
    - ビルド成果をシェルに認識させる
        - `source ~/.bashrc`
- シェルを合計３つ立ち上げておく(２つ追加で立ち上げる)
    - 今後の操作で最大３つ必要になるので今のうちに立ち上げておく。新しいターミナルを開いて`./run.sh`を実行して、Docker環境に入る。

### 予備知識のご紹介
- macOSでは、DockerからOSのデスクトップ上に3D系のアプリを表示できない。この環境では、代わりにWEBブラウザ上に表示出来るようにしてある。その環境を開くには、 http://localhost:6080/ へアクセスし、VNCの環境に入る。
    - この環境でコマンドを打つことは基本的に無い。直下のおまじないが唯一の例外。
- シミュレータの実行に便利な"おまじない"の紹介。現在の実装だと、何故かsimが一分程度で操作できなくなる不具合がある。下記のコマンドを打つと何故か不具合が消える。turtlebot3_ign_gazebo.launch.pyの実行時に一緒に実行すると吉。**vncコンテナ環境の人はvnc側でターミナルを開いて実行する。**
    - `ros2 run ros_ign_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock@ignition.msgs.Clock`

### 地図作成
- シミュレータを起動
    - `ros2 launch cub_bringup turtlebot3_ign_gazebo.launch.py`
    - 必要に応じて、上記のおまじないを(別のシェルで)実行
    - VNCコンテナを使うmacOS勢は、これの動きがすっっっごい重いと思う。
- 地図作成ソフトウェアを起動(上記のシェルは既に使ってるので、別ので実行)
    - `ros2 launch cub_bringup turtlebot3_cartographer.launch.py`
- 地図を保存(上記のシェルは既に全部使ってるので、さらに別ので実行)
    - 地図を保存するディレクトリを用意、この場合はmap1ディレクトリに地図を突っ込む想定
        - `mkdir -p ~/maps/map1`
    - 地図を保存
        - `ros2 run nav2_map_server map_saver_cli -f ~/maps/map1/map --ros-args -p save_map_timeout:=1000.0`
        - ~/maps/map1/ 以下に、map.pgmとmap.yamlが生成される
        - map.pgmはgimp等で編集可能な画像ファイル
- 終了
    - 全部のターミナルでCtrl+Cを押して、プロセスを落として回る。

### 自律走行
- シミュレータを起動
    - `ros2 launch cub_bringup turtlebot3_ign_gazebo.launch.py`
    - 必要に応じて、上記のおまじないを実行
- 自律走行ソフトウェアを起動
    - `ros2 launch cub_bringup turtlebot3_navigation2.launch.py use_sim_time:=True map:=/home/cub/maps/map1/map.yaml`
- mapの方のrvizに地図出ない時は以下を実行(ign_gazebo.launch.pyの方のと合わせて２つ起動してるはず)
    - `ros2 lifecycle set map_server configure && ros2 lifecycle set map_server activate`
- 自律走行
    - turtlebot3を使ったサンプルはインターネットに沢山あるので、それを参考にするのが楽
    - 例えば[こちらの記事の「Navigation2の実行」の項目](https://qiita.com/porizou1/items/d63a41fc1e478dfa5ab6#navigation2%E3%81%AE%E5%AE%9F%E8%A1%8C)
    - ネット上の記事の通り、機体の初期姿勢を指定して、ゴールを指定するとその地点に走る。✌楽しい✌
- 終了
    - 全部のターミナルでCtrl+Cを押して、プロセスを落として回る。

### Docker環境の終了
- Docker環境の終了
  - exitコマンド等でコンテナから出て、`./stop.sh`を実行すると止まる。
