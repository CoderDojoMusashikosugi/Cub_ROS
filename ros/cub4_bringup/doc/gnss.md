# Jetson Orin Nano GNSS (NMEA + PPS) 時刻同期 調査・設定ガイド

Jetson Orin Nano の 40 ピン拡張ヘッダー（J12）のみを使用して、GNSS モジュールからの NMEA（シリアル）および PPS（Pulse Per Second）を入力し、高精度なシステム時刻同期を実現するための調査結果および設定手順です。

---

## 1. 対象システム環境

- **ハードウェア**: NVIDIA Jetson Orin Nano Developer Kit Super
  - キャリアボード: `p3768-0000`
  - SoM モジュール: `p3767-0005` (Orin Nano Super)
- **OS**: Ubuntu 24.04.4 LTS (Noble Numbat)
- **カーネル**: `Linux 6.8.12-1021-tegra aarch64`
- **L4T バージョン**: `39.2.0` (JetPack 6 系)
- **カーネル PPS サポート状況**:
  - `CONFIG_PPS=y` (有効)
  - `CONFIG_PPS_CLIENT_GPIO=y` (有効・ビルトイン)
  - `CONFIG_PPS_CLIENT_LDISC=y` (有効)
  - **結論: カーネルの再ビルドは一切不要。標準カーネルのまま PPS-GPIO が動作可能。**

---

## 2. 40 ピン拡張ヘッダー（J12）配線仕様

> [!WARNING]
> Jetson Orin Nano の 40 ピンヘッダーは **3.3V CMOS ロジック** です。
> GNSS モジュールの UART 出力および PPS 出力が 3.3V であることを確認してください。5V 出力のモジュールを直接接続すると SoC が破損する恐れがあるため、レベルシフタや抵抗分圧が必要です。

| 信号名 | GNSS モジュール側 | 40 ピンヘッダ側 | ピン番号 | 内部デバイス / 備考 |
| :--- | :--- | :--- | :--- | :--- |
| **電源** | VCC | 5V または 3.3V | **Pin 2** (5V) / **Pin 1** (3.3V) | モジュールの仕様に合わせて選択 |
| **GND** | GND | GND | **Pin 6** | Pin 9, 14, 20, 25, 30, 34, 39 でも可 |
| **NMEA (受信)** | TXD (GNSS出力) | UART RXD | **Pin 10** | **`/dev/ttyTHS1`** で受信 |
| **NMEA (送信)** | RXD (GNSS入力) | UART TXD | **Pin 8** | `/dev/ttyTHS1` (設定コマンド用。受信用途のみなら未接続可) |
| **PPS** | PPS (パルス出力) | GPIO | **Pin 32** | **`GPIO07` / `PG.06` (GPIO line offset: 54)** |

### ピン選定の根拠
- **UART (Pin 8 / 10)**: デバイスツリー上で `serial1` (`/bus@0/serial@3100000` / `tegra194-hsuart`) に割り当てられており、Linux 上では `/dev/ttyTHS1` として認識されます。ブート引数のシリアルコンソールは `ttyTCU0` に割り当てられているため、競合なく利用できます。
- **PPS (Pin 32 / GPIO07)**: 40ピンヘッダの信号名は `GPIO07`、SoC 側のピン名は `soc_gpio19_pg6`、メイン GPIO コントローラ `tegra234-gpio` における line offset は `54` です。`support_tools/jetson_pps/jetson-pps-gpio07.dts` にて定義されています。

---

## 3. デバイスツリーの変更（PPS-GPIO の有効化）

Linux 6.8 カーネルは `CONFIG_PPS_CLIENT_GPIO=y` が組み込まれているため、デバイスツリーオーバーレイ（DTBO）を追加するだけで `/dev/pps1` が自動生成されます。

### (1) オーバーレイ DTS ファイル (`support_tools/jetson_pps/jetson-pps-gpio07.dts`)

```dts
/dts-v1/;
/plugin/;

/ {
    overlay-name = "Jetson PPS GPIO Overlay (Pin 32 / GPIO07)";
    compatible = "nvidia,p3768-0000+p3767-0005-super", "nvidia,tegra234";

    fragment@0 {
        target = <&pinmux>;
        __overlay__ {
            pps_pins: pps_pins {
                hdr40-pin32 {
                    nvidia,pins = "soc_gpio19_pg6";
                    nvidia,function = "gp";
                    nvidia,tristate = <0x01>;      /* 入力モード */
                    nvidia,enable-input = <0x01>;  /* 入力バッファ有効化 */
                };
            };
        };
    };

    fragment@1 {
        target-path = "/";
        __overlay__ {
            pps {
                compatible = "pps-gpio";
                pinctrl-names = "default";
                pinctrl-0 = <&pps_pins>;
                gpios = <&gpio 54 0>;              /* 54 = TEGRA234_MAIN_GPIO(G, 6) = PG.06 (Pin 32 / GPIO07), 0 = ACTIVE_HIGH */
                status = "okay";
            };
        };
    };
};
```

### (2) DTBO のコンパイルと `/boot` への配置

```bash
# DTSをコンパイルして /boot に配置
sudo dtc -@ -I dts -O dtb -o /boot/jetson-pps-gpio07.dtbo jetson-pps-gpio07.dts
```

### (3) ブートローダー (`/boot/extlinux/extlinux.conf`) への登録

`/boot/extlinux/extlinux.conf` の `LABEL primary` エントリに、ベース DTB (`FDT`) とオーバーレイ (`OVERLAYS`) を指定します。

```text
TIMEOUT 30
DEFAULT primary

MENU TITLE L4T boot options

LABEL primary
      MENU LABEL primary kernel
      LINUX /boot/Image
      INITRD /boot/initrd
      FDT /boot/dtb/kernel_tegra234-p3768-0000+p3767-0005-nv-super.dtb
      OVERLAYS /boot/jetson-pps-gpio07.dtbo
      APPEND ${cbootargs} root=PARTUUID=... rw rootwait rootfstype=ext4 ...
```

システム再起動後、PPS クライアントとして `/dev/pps1`（または `/dev/pps0`）が生成されます。

---

## 4. 時刻同期ソフトウェア（NTPsec）の導入と設定

Chrony はシリアルポートから直接 NMEA センテンスをパースする機能を備えておらず `gpsd` 等の中間デーモンが必要になります。一方、**NTPsec**（Ubuntu 24.04 公式パッケージ）はシリアルポートから直接 NMEA を解析して PPS パルスと直結するビルトインドライバ（`refclock nmea`）を備えているため、**余計なデーモンを常駐させることなく単体で高精度な完全オフライン時刻同期を実現**できます。

### (1) パッケージのインストールとアクセス権限・AppArmor 設定

```bash
# 競合サービス（systemd-timesyncd, chrony）の停止・無効化
sudo systemctl stop systemd-timesyncd chrony
sudo systemctl disable systemd-timesyncd chrony

# ntpsec および動作確認ツールのインストール
sudo apt update
sudo apt install -y ntpsec pps-tools

# ntpsec ユーザーを dialout グループに追加（シリアルポート・PPSへのアクセス権付与）
sudo usermod -aG dialout ntpsec

# PPS デバイスのパーミッション設定（udev）
echo 'KERNEL=="pps*", GROUP="dialout", MODE="0660"' | sudo tee /etc/udev/rules.d/99-pps.rules
sudo udevadm control --reload-rules && sudo udevadm trigger

# AppArmor によるシリアルポートアクセスのブロックを解除
echo "/dev/ttyTHS[0-9]* rw," | sudo tee -a /etc/apparmor.d/local/usr.sbin.ntpd
sudo apparmor_parser -r /etc/apparmor.d/usr.sbin.ntpd
```

### (2) NTPsec 設定 (`/etc/ntpsec/ntp.conf`)

`/etc/ntpsec/ntp.conf` に以下の `refclock nmea` 行を追記します：

```conf
# --- Cub GNSS Time Sync Settings ---
# NMEAシリアル (/dev/ttyTHS1) と PPSパルス (/dev/pps1)
# flag1 1: PPS処理の有効化, prefer: 同期確立時に最優先ソースとして採用
refclock nmea path /dev/ttyTHS1 ppspath /dev/pps1 baud 9600 flag1 1 prefer
# --- End of Cub GNSS Time Sync Settings ---
```

設定反映：
```bash
sudo systemctl restart ntpsec
```

> **💡 自動化スクリプトでの一括適用:**  
> 上記のSwap、PPSオーバーレイ、NTPsec設定、AppArmor、udev設定はすべて `scripts/install_host_settings.sh` に実装されています。`sudo ./scripts/install_host_settings.sh` を実行するだけで全自動で適用されます。

---

## 5. 動作確認・検証手順

### ① PPS パルスの着信確認 (`ppstest`)
```bash
sudo ppstest /dev/pps1
```
**期待される出力例:**
```text
trying PPS source "/dev/pps1"
found PPS source "/dev/pps1"
ok, found 1 source(s), now start fetching data...
source 0 - assert 1726111234.000000123, sequence: 10 - clear  0.000000000, sequence: 0
source 0 - assert 1726111235.000000115, sequence: 11 - clear  0.000000000, sequence: 0
```
*(1秒ごとに assert タイムスタンプが更新されれば正常)*

### ② NMEA データの受信確認
```bash
sudo tio /dev/ttyTHS1 -b 9600
# または
sudo cat /dev/ttyTHS1
```
`$GNRMC`, `$GNGGA` などの NMEA センテンスが定期的に流れてくれば正常。

### ③ NTPsec の同期ステータス確認
```bash
ntpq -pn
```
**期待される出力:**
```text
     remote           refid      st t when poll reach   delay   offset   jitter
=============================================================================
*                    .GPS.        0 l    -   64  377   0.0000  -0.0012   0.0015
+                    133.243.x.x  2 u   59   64  377  75.2881  -0.5356  23.9514
```
- `.GPS.` 行の先頭に `*`（マスタークロックとして選択中）または `+` が付き、`reach` が `377`（正常受信）となることを確認。
- ※GPSモジュール未接続時は、外部NTPサーバ側の行に `*` が付き、インターネット経由で同期します。

システムクロックの変数の確認：
```bash
ntpq -c rv
```
`leap=00`、`status=... clock_sync` となっていれば、システム時刻の同期が完了しています。

---

## 6. Docker コンテナ内プロセスからの時刻同期ステータス確認

Cub_ROS は `docker-compose-common.yml` にて `network_mode: "host"` を利用しているため、**ボリュームマウントを追加することなく、ホストネットワーク経由（127.0.0.1）でホストの NTPsec デーモンに問い合わせが可能**です。

> **💡 ボリュームマウントを行わないメリット:**  
> ホスト側に NTPsec が導入されていない環境（シミュレーション専用PC等）で同じ `docker-compose.yml` を利用した場合でも、ホスト上に不要なディレクトリが勝手に作られたり、起動時マウントエラーが発生するリスクを防ぎ、安全に設定を共有できます。

### (1) コンテナ内 CLI での確認手順

コンテナ内に `ntpsec`（`ntpq` コマンド）が入っていれば、ホストの IP（`127.0.0.1`）を指定して実行できます：

```bash
# 同期ピアの一覧とアクティブソース（*）の確認
ntpq -pn 127.0.0.1

# システムクロック同期変数の詳細確認
ntpq -c rv 127.0.0.1
```

**同期ステータスの判定ポイント:**
- `.GPS.` 行の先頭に `*`（sys.peer）：GNSS (NMEA + PPS) によるナノ秒オーダー高精度同期中
- 外部 IP アドレスの先頭に `*`：外部 NTP による同期中
- `ntpq -c rv` の `leap=00`：システム時刻同期中（`leap=11` は未同期）

### (2) Python / ROS 2 ノードからの取得実装例

ROS 2 ノード（自己位置推定やセンサー同期ノード等）から現在時刻の同期状態を把握するスクリプト例です。デーモン停止時や非同期時でも例外を出さずに安全にハンドリングできます。

```python
import subprocess

def get_time_sync_status():
    """
    ホストの NTPsec ステータスを取得し、同期元と誤差を判定する
    """
    try:
        # ntpq -c rv 127.0.0.1 でシステムクロック変数を取得 (タイムアウト 1秒)
        res = subprocess.run(
            ["ntpq", "-c", "rv", "127.0.0.1"],
            capture_output=True,
            text=True,
            timeout=1.0
        )
        if res.returncode != 0:
            # NTPsec デーモン未起動または接続拒否時
            return {"synced": False, "source_type": "NONE", "reason": "daemon not reachable"}

        # "key=value, key=value" 形式の出力をパース
        status = {}
        for token in res.stdout.replace("\n", "").split(","):
            token = token.strip()
            if "=" in token:
                k, v = token.split("=", 1)
                status[k.strip()] = v.strip().strip('"')

        leap = status.get("leap", "11")
        refid = status.get("refid", "")
        offset = status.get("offset", "0.0")

        # leap=00 の場合は同期確立済み
        is_synced = (leap == "00")

        # 同期ソースの判定
        if ".GPS." in refid:
            source_type = "GNSS_PPS"
        elif is_synced:
            source_type = "EXTERNAL_NTP"
        else:
            source_type = "UNSYNCED"

        return {
            "synced": is_synced,
            "source_type": source_type,
            "refid": refid,
            "offset_ms": offset,
        }

    except (subprocess.TimeoutExpired, FileNotFoundError):
        return {"synced": False, "source_type": "NONE", "reason": "command failed or ntpq not installed"}
```

