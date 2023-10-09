# install
* [PS5 Controller](https://github.com/rodneybakiskan/ps5-esp32)
* [DDT Motor](https://github.com/takex5g/M5_DDTMotor_M15M06)
* ROS
  1. スケッチ ⇒ ライブラリをインクルード ⇒ ライブラリを管理
  2. 『rosserial』と検索
  3. Michael Furguson 氏の『Rosserial Arduino Library』をインストール
  4. ros.hの修正が必要[リンク先参照](https://github.com/espressif/arduino-esp32/issues/4807)
  5. rosserialがあるフォルダから、Document/Arduino/libraries/Rosserial_Arduino_Library/src/ros/node_handle.hのサイズを1024に変更（nav_msgs::Odometryのサイズ調整のため）
     ```
     /* Node Handle */
     template<class Hardware,
         int MAX_SUBSCRIBERS = 25,
         int MAX_PUBLISHERS = 25,
         int INPUT_SIZE = 1024,
         int OUTPUT_SIZE = 1024>
     ```
