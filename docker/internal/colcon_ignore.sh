# 機体の特性上使わないパッケージに対しCOLCON_IGNOREを配置してビルド対象から外すためのスクリプト

SCRIPT_DIR=$(cd $(dirname $0); pwd)
BASE_DIR=$SCRIPT_DIR/../..
ENV_DIR=$BASE_DIR/docker/environment
ROS_DIR=$BASE_DIR/ros


# 引数があればそれをtargetに設定、無ければtarget.envの値を採用
# 引数がallである場合は全部のパッケージをビルドする設定とする。
source $BASE_DIR/target.env
target="${1:-$CUB_TARGET}"

# all以外の場合はターゲットの設定を読み込む
if [ $target != "all" ]; then
  source $ENV_DIR/$target.conf
fi
if [ $? = 1 ]; then
  # 読み込めない場合はエラーを出す。
  echo ""
  echo "target.envのCUB_TARGETに設定された[" $target "]は、設定ファイル群の中に存在しません。"
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
target_ros_package_exclusive=("${ros_package_exclusive[@]}")
target_ros_package_disable=("${ros_package_disable[@]}")

# echo target_ros_package_exclusive ${target_ros_package_exclusive[@]}
# echo target_ros_package_disable ${target_ros_package_disable[@]}

# 自身の設定でdisableのパッケージは使わない
self_disable=("${target_ros_package_disable[@]}")

# 他の設定でexclusive登録されたパッケージも使わない
files="$ENV_DIR/*.conf"
for filepath in $files; do
  # echo $filepath
  if [ $filepath = $ENV_DIR/$target.conf ] ; then
    # echo "this is the target"
    :
  else
    source $filepath
    for ((i=0; i<${#ros_package_exclusive[@]}; i++))
    do
      # echo "\${ros_package_exclusive[$i]}=${ros_package_exclusive[$i]}"
      self_disable+=(${ros_package_exclusive[$i]})
    done
  fi
done

# 重複を消す
self_disable=(`echo "${self_disable[*]}" | tr ' ' '\n' | sort -u`)

# 自身の設定でexclusiveのパッケージは使うのでリストから削除
disable=() # この結果が最終的にdisableするパッケージのリスト
for ((i=0; i<${#self_disable[@]}; i++))
do
  if printf '%s\n' "${target_ros_package_exclusive[@]}" | grep -qx ${self_disable[$i]}; then
      # echo ${self_disable[$i]} "is exclusive"
      :
  else
    disable+=(${self_disable[$i]})
    # echo ${disable[@]}
  fi
done

# もしtargetがallだった場合は今までの話を完全無視して全部のパッケージを有効化する
if [ $target = "all" ]; then
  disable=()
fi

# 手元の全てのrosパッケージのパスを取得
ROS_PATHS=$(find $ROS_DIR -name package.xml)

# それぞれdisableのリストにあるかを照合して、COLCON_IGNOREを配置
while read package_xml_path; do
  # パッケージ名を取得
  package_name=$(grep '<name>' "$package_xml_path" | sed -n 's:.*<name>\(.*\)</name>.*:\1:p')
  package_dir=$(dirname $package_xml_path)

  ignore_this_package=0
  for ((i=0; i<${#disable[@]}; i++))
  do
    # echo "\${ros_package_exclusive[$i]}=${ros_package_exclusive[$i]}"
    if [[ $package_name == ${disable[$i]} ]]; then
      # echo $package_name "と" ${disable[$i]} "は一緒"
      ignore_this_package=1
    fi
  done

  colcon_ginore_path=$package_dir/COLCON_IGNORE

  # ファイルの存在を0,1で取得
  [ ! -f $colcon_ginore_path ]
  file_exists=$?

  # 全てのrosパッケージに対して、disableリストにあればCOLCON_IGNOREを置く。リストに無ければ消す。
  # COLCON_IGNOREの存在と、このパッケージを無視するかの判定が食い違ってたら是正
  if [ $(( file_exists ^ ignore_this_package )) -eq 1 ]; then
    # 違ってたら是正
    if [ $ignore_this_package = 1 ]; then
      echo "add COLCON_IGNORE" $package_name # "     " $package_dir
      touch $colcon_ginore_path
    else
      echo  "delete COLCON_IGNORE" $package_name # "     " $package_dir
      rm -f $colcon_ginore_path > /dev/null
    fi
  else
    # echo  $ignore_this_package $package_name "     " $package_dir
    : # 現状と合ってたら何もしない
  fi

done <<< "$ROS_PATHS"


