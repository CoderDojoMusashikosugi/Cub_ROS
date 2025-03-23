# docker
## Docker環境系の技術情報
### サポート環境
(≠実行した実績のある環境)

- CPU Arch
  - arm64 (RaspberryPi, Jetson Orin Nano, Apple Silicon Mac等)
  - x86_64 (UbuntuブートのPCやWSLg環境等)
- OS
  - NVIDIA Jetpack (6を想定しているが、それ以外でも大抵大丈夫)
  - Ubuntu (22.04を想定しているが、それ以外でも大抵大丈夫)
  - RaspberryPi OS 64bit (RasPi5を想定しているが、4でも大丈夫)
  - WSLg上のUbuntu (22.04を想定しているが、それ以外でも大抵大丈夫)
  - macOS (Apple Silicon Macを想定しているが、IntelMacでも大丈夫)
- GPUアクセラレーション
  - NVIDIAのGPUを搭載した環境では、NVIDIA Container Toolkitが有効であればCUDAが使える

### Dockerイメージの構造
- ベースのイメージはnvidia/cuda
- **Baseイメージ**(cub_ros_base)は、↑これにROS等必須のパッケージを追加して作成される
- **(通常の)コンテナイメージ**(cub_ros)は、↑これに`ros`以下のパッケージが要求するaptパッケージや`docker/additional_pkgs.bash`でインストールするパッケージを追加したもの。通常はこれが起動される。
  - ベースイメージが存在するのは、無いとadditional_pkgsを編集する度にROSまるごとインストールからやり直されて非常に時間がかかり面倒であったため。
- **VNCイメージ**(cub_ros_vnc)は、↑これに[Tiryoh/docker-ros-desktop-vnc](https://github.com/Tiryoh/docker-ros-desktop-vnc)の機能を追加したもの。macOS環境ではこれが起動される。
  - この構造がゆえ、additinal_pkgsを編集する度にVNCのインストールからやり直されて非常に時間がかかり面倒。mac以外の環境で動作確認してから作成するのがおすすめ。

- Dockerイメージのタグは、「バージョン名_アーキテクチャ名」で付けられる。
  - バージョン名は、その日の日時を秒まで入れたもの。数人程度なら同時に開発してても、さすがに秒までは被らないかなと思って・・・
  - アーキテクチャ名はarm64またはx86_64が入る。
  - このタグ付けに特に深い意味は無いし、アーキテクチャごとにタグ分けなくて良い仕組みの活用や、日時基準の甘いID設定の撤廃等は時間があればやっても良さそう。

### ファイル構造と簡単な説明
- run.sh: 一度叩くだけで色々実行してDockerコンテナ内のbashを起動するスクリプト。
  - コンテナが起動していない場合
    - 最初にDocker環境をpullしようとする
    - pullが出来なければ、代わりにbuildする
    - 上の作業でDockerイメージが用意できたら、runする
    - 最後にexecしてbashを起動
  - コンテナが起動している場合
    - execしてbashを起動
  - 実際には上記の動作は./docker/run.shに書いてあって、それを呼び出しているだけ。
- stop.sh: Dockerコンテナを落とすスクリプト。
  - 実際には動作は全部./docker/stop.shに書いてあって、それを呼び出しているだけ。
- target.env: ロボットの種類を設定する用ファイル
- ros/: ここにrosのパッケージを置く。以下は配置例。コンテナ内では`~/colcon_ws/src/cub/`にマウントされる。
  - cub_bringup
    - package.xml
- docker/: Docker関連のファイルを置いている。
  - .default_do_not_edit: この中のファイルをコピーして配置する際のコピー元として使っているディレクトリ
  - home/: この中に置いたファイルはコンテナのホームディレクトリにマウントされる。
    - .user_config.bash: git pull直後には無いが、./run.shを実行したら最初だけ生成されるファイルでその後は消されない。個人用の設定を入れておく用。ROSのPC間通信設定などに。
    - .bashrc: ↑を含め色々sourceする部分が追加されている。あとcb,cbs,bashrc等の便利aliasが登録されている。
  - additional_pkgs.bash: dockerイメージにインストールしたいパッケージを書くなどに使う。build.shでイメージ生成時に実行される。
  - docker-compose.yml & docker-compose-common.yml: デバイスとの接続設定などに使う。commonに通常とVNCのコンテナの共通部分を書いて、差分をdocker-compose.ymlに書く。
  - .env: Docker向けの設定を記載する。今のところユーザー名しか書いてない。
  - ver_base.env & ver_vnc.env & ver.env: 起動するDockerイメージのバージョンを保存する。
  - Dockerfile & Dockerfile_vnc & Dockerfile.base: コンテナごとのDockerfile。追加したいものがあれば主にadditional_pkgs.bashに書けば良いため、あまり触る機会は無い。
  - install.sh: ホスト環境にDockerをインストールするスクリプト。無理そうなら諦めたり解説へのリンクを教えてくれたりする。
  - build.sh: 現在の設定に従って新たなベースイメージを生成するスクリプト。additional_pkgs.bashを書き換えた場合などは、これを実行すると適用される。
  - login.sh: ghcr.ioにログインする。ログイン情報はホスト環境の~/.netrcに置かれたgithubのトークンを利用する。ここに正しい権限をつけたトークンを置く必要があるのには注意。
  - push.sh: dockerコンテナをghcr.ioにpushする。
  - remove.sh: cub_ros_base/cub_ros/cub_ros_vncそれぞれで、最新以外のイメージを全部削除する。容量削減に効果的。最新というのはdocker imagesで上の方に来るものを差しているだけで、今実際レポジトリが指定するバージョンであるかを見てないのには注意。
  - run.sh: ./run.shの実体。解説はその項目を参照。
  - stop.sh: ./stop.shの実体。解説はその項目を参照。
  - README.md: これ。
  - internal: Docker関係のスクリプトを置いているディレクトリ。この中のスクリプトを直接叩く機会はあまり無いと思う。稀に直接叩くものには★をつけた。
    - ★base_build.sh: 現在の設定に従って新たなベースイメージを生成するスクリプト。これを使う機会は極めて少ないので、間違えて実行しないようにinternalに置いた。
      - ./docker/build.shでは通常イメージしかビルドせず、ベースイメージはそのまま。ベースイメージから生成し直したい場合はこれを実行後に./docker/build.sh。
    - container_install_docker.sh: DockerイメージにDockerをインストールするためのスクリプト。今は意味がないがかつてDooDに使っていた。
    - container_install_prebuilt.sh: 主にaptで入らないROSパッケージのビルドをDocker Build中に実施するもの。変更を加える必要なければここに置くと便利。ARM向けのバイナリ無いシミュレータ等はここに置くと便利。
    - container_install_ros.sh: コンテナイメージにROSをインストールするスクリプト。
    - docker_exec.sh: コンテナでコマンドを実行する。例えば'docker/docker_exec.sh /bin/bash'でシェルが立ち上がる。./run.shはコンテナ起動後にこれを呼び出す。
    - docker_install_raspi.sh: ./docker/install.shで、環境がRasPiだと判定されたらこれが呼び出される。
    - docker_install_ubuntu.sh: ./docker/install.shで、環境がUbuntuだと判定されたらこれが呼び出される。
    - docker_util.sh: docker-composeのための設定を読み込む。特に.env系のファイルなど。
    - docker-compose-nvidia.yml: NVIDIA Container Toolkitを利用する際に、docker composeのoverride機能を利用してこのファイルの設定を追加する。
    - entrypoint_vnc.sh: VNCコンテナ用のentrypoint。イメージ作成時にコピーされるので、書き換えだけでは動作は変化しない。
    - entrypoint.sh: VNCコンテナ用のentrypoint。イメージ作成時にコピーされるので、書き換えだけでは動作は変化しない。
    - ★vnc_build.sh: 現在の設定に従って新たなVNCイメージを生成するスクリプト。これを使う機会は極めて少ないので、間違えて実行しないようにinternalに置いた。
      - Macでは./docker/build.shでVNCイメージもビルドされるが、他の環境では要らないのでビルドしない。これをUbuntu等でも無理やりビルドするスクリプトがこれ。
- README.md: README。
- m5atom: Docker的には関係ない。M5 Atom向けのスクリプト置き場。
- support_tools: Docker的には関係ない。今のところDynamixel関係のファイルが置かれている。
- scripts: Docker的には関係ないスクリプト置き場。ホスト環境のセットアップ用スクリプトなど。
  - install_host_settings.sh: Cub2用
  - mcub_host_settings.sh: mCub用
  - libuvc_installation.sh: ?

### Dockerコンテナの構造
- `docker/home`は、コンテナ内でホームディレクトリにマウントされている。
- `support_tools`は、コンテナ内で`~/support_tools`にマウントされている。
- `ros`は、コンテナ内で`~/colcon_ws/src/cub`にマウントされている
  - このため、コンテナ内の`~/colcon_ws/src/`に何か置いてもgitレポジトリには入らない。また、ホスト側の`docker/home/colcon_ws/src/cub`に何か置いてもコンテナ内からは見えない。
  - ROSのコードを編集したければ、`ros`ディレクトリ内のパッケージを編集すれば良い。

## 使い方
### Dockerコンテナに新しいaptパッケージを追加したい(ROSとは関係なく)
1. docker/additional_pkgs.bash にインストールする命令を追加する。
2. `./docker/build.sh`でコンテナを更新する
3. これを共有するなら、docker/ver.envをgit commit & pushして、さらにdockerイメージも./docker/login.sh & ./docker/push.shでプッシュする。
   - ./docker/login.shについてはファイル構造の項目を参照

### DockerコンテナにROSノードで使用する新しいライブラリを追加したい
実はadditinal_pkgs.bashを使わずとも実行可能、というかそれがおすすめ

1. ROSパッケージのpackage.xmlに、使いたいライブラリを`<depend>`タグ等で指定する
2. `./docker/build.sh`でコンテナを更新すると、内部でrosdepがパッケージが必要とするライブラリを探してインストールしてくれる
3. あとはaptパッケージの項目と同じ

インストールするパッケージのリストをadditional_pkgs.bashにだけ書くと、どのパッケージがどのライブラリを追加したがってるのか分からなくなる。こうしておくと対応付けが楽になる。

### Dockerコンテナに予めビルド済みのパッケージを追加したい
micro_rosの追加の際に必要になった機能

1. docker/internal/container_install_prebuilt.sh にインストールする司令を追加する
2. あとはaptパッケージの項目と同じ

