# CH341 USB-Serial Driver for Linux

本ディレクトリには、WCH CH340 / CH341 USB-Serial 変換チップ（UM982 GNSSモジュール等で使用）向けのカーネルドライバソースコードおよびビルド用 Makefile を配置しています。

Jetson（Linux for Tegra / JetPack 6.x）の標準カーネル設定では `CONFIG_USB_SERIAL_CH341` が有効化されていないため、本ソースコードを用いてカーネルモジュール（`ch341.ko`）をビルドして利用します。

## 取得元情報

- **リポジトリ**: [torvalds/linux (GitHub)](https://github.com/torvalds/linux)
- **ファイルパス**: `drivers/usb/serial/ch341.c`
- **対象タグ**: `v6.8`
- **取得元URL**: [https://raw.githubusercontent.com/torvalds/linux/v6.8/drivers/usb/serial/ch341.c](https://raw.githubusercontent.com/torvalds/linux/v6.8/drivers/usb/serial/ch341.c)
- **ライセンス**: GPL-2.0

## 手動ビルド・インストール手順

通常は `scripts/install_host_settings.sh` を実行することで自動的にビルド・インストールされます。
手動でビルド・インストールする場合は以下の手順を実行してください。

```bash
# ビルド
make

# インストール
sudo mkdir -p /lib/modules/$(uname -r)/kernel/drivers/usb/serial
sudo cp ch341.ko /lib/modules/$(uname -r)/kernel/drivers/usb/serial/
sudo depmod -a

# モジュールロード
sudo modprobe ch341

# 起動時自動ロードの設定
echo "ch341" | sudo tee /etc/modules-load.d/ch341.conf
```
