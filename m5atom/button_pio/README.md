# M5Dial USB NCM / micro-ROS button

M5Dialの物理ボタンの状態を、USBネットワーク（NCM）上のUDPでmicro-ROS Agentへ送り、
ROS 2 Jazzyのトピックとして配信する。Wi-Fiは使用しない。

```text
M5Dial ── USB NCM / UDP ── PC ── micro-ROS Agent ── ROS 2
       DHCPサーバー         DHCPクライアント
```

## 機能

- ノード `/m5dial_button` から `/dial/button`（`std_msgs/msg/Bool`）を配信する。
  物理ボタンの押下中は `true`、解放中は `false`。状態変化時と100msごとに送信する。
- 押した瞬間に4000Hz・50msのブザーを1回鳴らす。長押し中や解放時は鳴らさない。
  ブザーはROSの接続状態に依存せず動作し、画面タッチはボタン操作として扱わない。
- 画面にボタン状態とROSの接続状態を表示する。下端にはビルド日付と時刻（JST）を
  小さく2行で表示する。現在時刻ではなく、書き込んだファームウェアのビルド日時であり、
  ビルドログの `M5Dial build:` と照合できる。
- Agentの停止・再起動やUSBの再接続後は、自動でROSへ再接続する。
  応答しなくなった旧セッションは、10秒の切断監視で削除する。
- 通常時のUSBはNCMのみ。ファームウェア更新はUSBネットワーク経由のOTAで行う。
  OTA認証は使用せず、パスワード設定は不要。更新中は `UPDATING` と表示し、ROS送信を
  一時停止する。更新失敗時はROS送信を再開する。

### ネットワーク設定

| 項目 | 設定 |
| --- | --- |
| M5DialのIP | `192.168.77.1/24` |
| PCのIP | M5DialのDHCPサーバーから `192.168.77.2`〜`192.168.77.3` を取得 |
| micro-ROS Agent | PCのUDP `8888`。M5DialはDHCPで払い出したPCのIPへ接続 |
| OTA更新要求 | M5DialのUDP `3232` |
| OTA転送 | M5DialがPCのTCP `3232` へ接続して受信 |
| ROSドメイン | `DIAL_ROS_DOMAIN_ID=0` |

PC側の固定IP設定やmDNSは不要。DHCPはデフォルトルートを通知しない。
通知されるDNSは、次のLinux設定で無視する。

## 開発環境の準備

VSCodeのPlatformIO拡張、またはPlatformIO Coreを使用する。
ビルドには `git`、CMake 3.22以上かつ4未満、`make` が必要。
初回ビルドではmicro-ROS JazzyのソースとPython依存パッケージを取得する。

以下の `pio` コマンドは、この `button_pio` ディレクトリで実行する。
VSCodeのPlatformIOターミナルでも実行できる。

### LinuxのUSBネットワーク設定

M5DialをUSB接続し、NetworkManagerで対象のUSBネットワークをDHCP（自動）にする。
`nmcli device status` でインターフェース名を確認する。
USB接続をDNSやデフォルトルートとして使わないよう、初回に次を設定する。
`DIAL_USB_IF` は実際の名前へ置き換える。

```bash
nmcli device status
DIAL_USB_IF=enx4a27e2e31fc5
DIAL_USB_CONNECTION="$(nmcli -g GENERAL.CON-UUID device show "$DIAL_USB_IF")"
sudo nmcli connection modify "$DIAL_USB_CONNECTION" ipv4.never-default yes ipv4.ignore-auto-dns yes
sudo nmcli device reapply "$DIAL_USB_IF"
```

ESP-IDFのDHCPサーバーはDNSオプションを無効にしても自身のIPをDNSとして通知するため、
PC側で無視する必要がある。
[DHCPサーバーの実装](https://github.com/espressif/esp-idf/blob/v5.5.5/components/lwip/apps/dhcpserver/dhcpserver.c#L469-L492)を参照。

### macOS（Apple Silicon）の準備

macOSではUSBによる書き込みを使う。書き込みにOTA、USB NCMのIP設定、Agentの起動は不要。
Homebrew導入済みの環境で、必要なツールをインストールする。

```bash
xcode-select --install
brew install git binutils
```

VSCodeにPlatformIO拡張を導入し、PlatformIOのPython環境へCMakeを入れる。
以下は標準インストール先の例。

```bash
~/.platformio/penv/bin/python -m pip install 'cmake>=3.22,<4'
```

GNU binutilsは `/opt/homebrew/opt/binutils/bin/` を使用する。
プロジェクトとPlatformIOの保存先には、空白を含まないパスを使う。
macOSの対応対象はApple Siliconのみ。macOS実機でのビルド・書き込みは未検証。
依存条件は [micro-ROS公式README](https://github.com/micro-ROS/micro_ros_platformio#platform-specific-requirements)も参照。

## ビルドと書き込み

| PlatformIO環境 | 用途 | 書き込み方法 |
| --- | --- | --- |
| `m5stack-dial`（既定） | 通常の更新 | USB NCM経由のOTA |
| `m5stack-dial-usb` | 初回導入・復旧、macOSでの書き込み | ROMのUSBシリアルとesptool |

### 通常の更新（OTA）

1. OTA対応ファームウェアが動いているM5DialをUSB接続し、PCがDHCPでIPを取得していることを確認する。
2. VSCodeで `button_pio` を開いてソースを編集する。
3. PlatformIOの `m5stack-dial` のUploadを実行する。
4. 再起動後、画面のビルド日時をビルドログと照合する。ROS接続を使う場合は受信も確認する。

ターミナルでは次を実行する。Uploadはビルドも行うため、Buildだけの事前実行は不要。

```bash
# ビルドのみ
pio run -e m5stack-dial

# ビルドして書き込み
pio run -e m5stack-dial -t upload
```

OTAはAgent停止中も使用でき、G0やリセットボタンの操作は不要。
アプリを別のOTA領域へ転送し、保存・検証後に再起動する。
書き込み先は `platformio.ini` の `upload_port`。
PCにファイアウォールがある場合は、USB側からPCのTCP `3232` への接続を許可する。

再起動後はDHCPとROSへ自動再接続する。
ただしハブ経由でUSBの再認識が止まる場合は、後述の「USBハブ経由で接続が戻らない場合」を参照。
PlatformIOの `SUCCESS` は転送成功を示し、再起動後の接続復帰までは保証しない。

### 初回導入・復旧（USB）

アプリやNCMが起動しない場合にも、ROMの書き込みモードから再書き込みできる。
bootloaderやパーティション表を変更する場合も、この方法を使う。

1. M5DialをPC本体のUSBポートへ直結する。
2. BTN0（G0 / BOOT）を押したままリセットを押して離し、その後BTN0を離す。
3. VSCodeのPlatformIOで `m5stack-dial-usb` のUploadを実行する。

ターミナルから行う場合:

```bash
pio run -e m5stack-dial-usb -t upload
```

ポートはOSごとに自動検出する。複数候補がある場合は `pio device list` で確認し、
`--upload-port` で指定する。macOSの例（`XXXX` は実際の名前へ置き換える）:

```bash
pio device list
pio run -e m5stack-dial-usb -t upload --upload-port /dev/cu.usbmodemXXXX
```

終了後はwatchdog-resetでアプリへ戻る。
通常動作ではUSBがNCMのみになり、シリアルポートが消えるのは正常。
ROMのUSBシリアルはアプリとは独立している。
[EspressifのBoot Mode Selection](https://docs.espressif.com/projects/esptool/en/latest/esp32s3/advanced-topics/boot-mode-selection.html)も参照。

## Cub_ROSと接続する

以下のコマンドは **Cub_ROSのリポジトリ直下** で実行する。
`cub_ros` コンテナはホストネットワークを共有する。

公式のmicro-ROS Agent実行ファイルをそのまま使用できる。
`cub4_bringup` の `launch_at_boot.launch.py` では、UDP用の `micro_ros_agent_dial` が
他のノードと一緒に起動・終了する。既存のシリアル用Agentとは別プロセスで共存する。

Agentだけを手動起動する場合は次を実行する。
launch側ですでに起動している場合は、同じUDPポートで二重起動しない。

```bash
./docker/run_in_container.sh 'ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 -v4'
```

別の端末で受信する。

```bash
./docker/run_in_container.sh 'ros2 topic echo /dial/button std_msgs/msg/Bool'
```

`platformio.ini` の `DIAL_ROS_DOMAIN_ID` と、Agent・他ノードの `ROS_DOMAIN_ID` を合わせる。

### 受信テスト

Agentを起動した状態で、次を実行する。

```bash
docker cp m5atom/button_pio/scripts/ros2_smoke_test.py cub_ros:/tmp/m5dial_smoke_test.py
./docker/run_in_container.sh 'python3 /tmp/m5dial_smoke_test.py --duration 20'
```

受信件数、周波数、状態変化、Publisher数を表示し、10件以上の受信とPublisherが1個であることを確認する。
押下と解放の両方を確認する場合は `--require-both` を付け、テスト中に物理ボタンを押して離す。

```bash
./docker/run_in_container.sh 'python3 /tmp/m5dial_smoke_test.py --duration 20 --require-both'
```

物理ボタンを押すと `true`、離すと `false` になり、押した瞬間だけ鳴ることを確認する。
画面を押し込まずにタッチしただけなら `false` のままで、ブザーも鳴らない。
Agentの停止・再起動後も同じテストで再接続を確認できる。

## USBハブ経由で接続が戻らない場合

一部のUSBハブでは、OTA転送が成功しても再起動後のUSB再認識が
`device descriptor read/64, error -110` などで止まる。
PC本体へ直結するか、Linuxホストで親ハブをソフトウェアリセットすると復旧できる。
リセットはUploadに自動では組み込んでいない。

ハブ配下の他のUSB機器も一時切断されるため、更新作業時に実行する。
シリアル機器を開いているノードが自動復帰しない場合は、そのノードも再起動する。

### 対象ハブの特定とリセット

1. `lsusb -t` でM5Dialの `Driver=cdc_ncm` を探す。
2. インデントを1段戻った、最も近い `Class=Hub` のバス番号とデバイス番号を確認する。
3. その番号で親ハブをリセットする。M5Dial自身は指定しない。

```text
Bus 001
    ... Class=Hub ...
        Port 002: Dev 055, Class=Hub, Driver=hub/4p
            Port 001: Dev 081, ... Driver=cdc_ncm
```

この例では親ハブは `001/055`。実際の番号に置き換えて実行する。
番号は再接続で変わる場合がある。

```bash
lsusb -t
lsusb -s 001:055
usbreset
sudo usbreset 001/055
```

同じVID:PIDのハブが1台だけなら、IDでも指定できる。
例えば `1a40:0101` のハブなら `sudo usbreset 1a40:0101`。

### 認識停止中でM5Dialがツリーに出ない場合

接続できていたときのUSB接続位置、または次のログに出る `usb 1-2.2.1` などを使う。

```bash
sudo dmesg --ctime | tail -n 50
```

NCMが認識されている間なら、インターフェース名から接続位置を確認できる。

```bash
readlink -f /sys/class/net/enx4a27e2e31fc5/device
```

末尾が `.../1-2.2.1/1-2.2.1:1.0` なら、M5Dialは `1-2.2.1`、親ハブは `1-2.2`。
親ハブの現在の番号を読み、リセット対象を特定する。

```bash
cat /sys/bus/usb/devices/1-2.2/busnum
cat /sys/bus/usb/devices/1-2.2/devnum
```

例えば `1` と `55` なら `sudo usbreset 001/055` を実行する。
PC本体へ直結している場合は、対象となる外付け親ハブはない。

## 設定変更とファイル構成

| ファイル | 役割 |
| --- | --- |
| `platformio.ini` | ビルド環境、依存ライブラリ、ROSドメイン、Agentポート、書き込み先 |
| `src/main.cpp` | 物理ボタン、ブザー、画面表示、micro-ROS Publisher |
| `include/usb_ncm.h` / `src/usb_ncm.cpp` | USB NCM、M5DialのIP、DHCPサーバー |
| `src/usb_ota.cpp` | OTA受信と更新状態の管理 |
| `microros.meta` | micro-ROSのリソース数と切断監視設定 |
| `scripts/prepare_microros.py` | 固定した公式micro-ROSライブラリのビルダーをPlatformIO 8とESP-IDFのassert設定に適合 |
| `scripts/build_timestamp.py` | ビルド日時ヘッダーを生成。増分ビルドでも更新し、IDE補完情報取得やクリーン処理では更新しない |
| `scripts/ros2_smoke_test.py` | コンテナ内でのトピック受信テスト |

既存ネットワークと `192.168.77.0/24` が重複する場合は、`include/usb_ncm.h` のIP、
`src/usb_ncm.cpp` のDHCP範囲とマスク、`platformio.ini` の `upload_port` を合わせて変更する。
Agentポートを変える場合は、`DIAL_AGENT_PORT` とAgentの `--port` を合わせる。

`microros.meta` を変更した場合は、ビルド前に両環境のmicro-ROSライブラリをクリアする。

```bash
pio run -e m5stack-dial -t clean_libmicroros
pio run -e m5stack-dial-usb -t clean_libmicroros
```
