# dynamixel
DynamixelのモータのIDを確認および変更をするツール。

# 使い方
## DynamixelSDKのインストール
コンテナ内で下記コマンドを実行
（こちらの操作はコンテナを再起動するたびに必要です）
```bash
cd ~/support_tools/dynamixel/DynamixelSDK/python
sudo python3 setup.py install
```

## 接続されているモータのID確認
コンテナ内で下記コマンドを実施
```bash
cd ~/support_tools/dynamixel/scripts
python3 01-broadcastPing.py
```
下記のような出力が出ます。(ID1とID2のモータが接続されている場合）
```bash
Dynamixel datalist  {1: [1190, 50], 2: [1190, 50]}
```

## モータのID変更
コンテナ内で下記コマンドを実行することで、ID1のモータをID2に変更できる
```bash
cd ~/support_tools/dynamixel/scripts
python3  02-change_id.py
```
