# これはdocker内に ~/.config.bash という名前で配置されるファイルで、.bashrcから読み込まれる。
# ここは開発者各個人で設定する内容を書いてもらうことを意図していて、.gitignoreに指定してコミット出来ないようにしてある。

# ROS_DOMAIN_IDは、ROS2を外部と接続する時の無線チャンネルのようなもので、同じ数字を指定して起動したノード同士で通信が可能。
# PC間でも同じ仕組みであるため、ロボットとPCを同じIDにすればPC間の通信が可能。
# 逆に、間違えて同じLANに居る人全員で同じIDで通信してしまうと干渉してうまく動けないはず。
# 値は概ね0から100の間で設定すると良いとされる。ROS2としてのデフォルトは0なので、そこと干渉しないようこのファイルでのデフォルト値は1とした。
export ROS_DOMAIN_ID=1

# FASTRTPS_DEFAULT_PROFILES_FILEは、他のPCとの通信向けの設定。設定ファイルのパスを指定する。
# デフォルトで設定されている /home/cub/.fastrtps_whitelist.xml の中には他のPCとの通信を禁止する内容が書いてある。
# なので、exportの方を設定すると他のPCとの通信ができなくなって、export -nの方を設定すると他のPCと通信できる。
# 普通はROS_LOCALHOST_ONLYでやるけど、VNC向けのコンテナ間通信も切れちゃうのでこちらで代替
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/cub/.fastrtps_whitelist.xml
# export -n FASTRTPS_DEFAULT_PROFILES_FILE

# export ROS_LOCALHOST_ONLY=1 # 一応、普通の設定の雛形も置いておく。これがオンだとMac向けのVNC環境のRVizにTopicが流れなくなるはず。


# 以上の内容を適用するには、全てのターミナルでdocker環境から抜けて入り直すのが確実
# または `source ~/.bashrc` を実行