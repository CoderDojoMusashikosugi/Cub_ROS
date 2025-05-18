#!/bin/bash

echo "This script setups host settings."
if [ $(whoami) != "root" ]; then
    echo "[error] run as sudo"
    echo "for example: sudo ./mcub_host_settings.sh"
    exit
fi

echo 'KERNEL=="ttyUSB*",  ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", SYMLINK+="ttyATOM"' > /etc/udev/rules.d/99-atom.rules
echo 'KERNEL=="ttyUSB*",  ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="ttySLC1"' > /etc/udev/rules.d/99-slc1.rules

POWER_CONFIG_FILE="/boot/firmware/config.txt"
if [ -z $(grep dtoverlay=gpio-shutdown $POWER_CONFIG_FILE) ]; then
    # GPIO20に接続されたmCubのSW31、Atom Matrix横のスイッチを押すと終了する設定。短押しで終了するはずだが、今のところ何故か一秒間隔で２回押す必要ある。
    echo "dtoverlay=gpio-shutdown,gpio_pin=20,active_low=1,gpio_pull=up" >> $POWER_CONFIG_FILE
fi
if [ -z $(grep dtoverlay=gpio-poweroff $POWER_CONFIG_FILE) ]; then
    # GPIO24に接続されたmCub基板中央LED30の緑色が、終了時に光るよう設定。終了ボタン押してから光るまでは結構時間かかる。
    echo "dtoverlay=gpio-poweroff,gpiopin=24,active_delay_ms=200,inactive_delay_ms=200,active_low=1" >> $POWER_CONFIG_FILE
fi

echo "setting completed. reboot to apply."
