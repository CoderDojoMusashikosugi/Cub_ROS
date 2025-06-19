# 初回起動時にtarget.envを配置するためのスクリプト
# 今設定してる内容が存在しない場合も教えてあげる

SCRIPT_DIR=$(cd $(dirname $0); pwd)
BASE_DIR=$SCRIPT_DIR/../..
ENV_DIR=$BASE_DIR/docker/environment
ROS_DIR=$BASE_DIR/ros

if [ -f $BASE_DIR/target.env ]; then
  source $BASE_DIR/target.env
  # 読んで設定がdocker/enbironmentに存在するかを確認する
  source $ENV_DIR/$CUB_TARGET.conf
  if [ $? = 1 ]; then
    # 読み込めない場合はエラーを出す。
    echo ""
    echo "target.envのCUB_TARGETに設定された[" $CUB_TARGET "]は、設定ファイル群の中に存在しません。"
    echo "CUB_TARGETを以下のいずれかに設定するか、./docker/environmentに新たな設定を追加してください。"
    echo "詳しくは、README.mdの「開発環境の用意」の項目を参照してください。"
    # 丁寧に設定の一覧を表示しておく。
    CONF_PATHS=$(find $ENV_DIR -name *.conf)
    echo "**********"
    while read CONF_PATH; do
      conf_name=$(basename $CONF_PATH)
      echo ${conf_name%.conf}
    done <<< "$CONF_PATHS"
    echo "**********"
    exit 1
  fi
else
  echo "# このレポジトリのソースコードをどのロボット向けにビルド・実行する予定かを設定するファイル。
# dockerを起動するとこれが読み込まれて環境変数に設定される。あとはビルドや実行の際にいい感じに使う。
# cub3/mcub/mcub_directなどから選ぶ。一覧には./docker/environment内の○○.confというファイル名を確認。
# このファイルは.gitignoreに設定してあるので、コミットされない。

CUB_TARGET=DEFAULT
" > $BASE_DIR/target.env

  echo "最初の起動のようですね。起動前に、初期設定の入力をお願いします。"
  echo "レポジトリ直下にtarget.envを配置しました。これをテキストエディタで開き、使いたいロボットの設定に書き換えてください。"
  echo "完了後、再度./run.shを実行してください。"
  exit 1
fi

