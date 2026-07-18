# これはdocker内に ~/.config.bash という名前で配置されるファイルで、.bashrcから読み込まれる。
# ここは開発者各個人で設定する内容を書いてもらうことを意図していて、.gitignoreに指定してコミット出来ないようにしてある。

# ROS_DOMAIN_IDは、ROS2を外部と接続する時の無線チャンネルのようなもので、同じ数字を指定して起動したノード同士で通信が可能。
# PC間でも同じ仕組みであるため、ロボットとPCを同じIDにすればPC間の通信が可能。
# 逆に、間違えて同じLANに居る人全員で同じIDで通信してしまうと干渉してうまく動けないはず。
# 値は概ね0から100の間で設定すると良いとされる。ROS2としてのデフォルトは0。
export ROS_DOMAIN_ID=0

# 他のPCと通信するかを選択。
# SUBNETにすると外部通信ON、LOCALHOSTでOFF
# シミュレーションをDocker外で動かす場合も外部通信扱いなのでSUBNETを設定
export ROS_AUTOMATIC_DISCOVERY_RANGE="SUBNET"
# export ROS_AUTOMATIC_DISCOVERY_RANGE="LOCALHOST"

# 以上の内容を適用するには、全てのターミナルでdocker環境から抜けて入り直すのが確実
# または `source ~/.bashrc` を実行