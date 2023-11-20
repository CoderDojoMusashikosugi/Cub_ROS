import serial
from datetime import datetime
import time

# 通信したいCOMポートを指定
com_port = "COM4"  # 例: 'COM3'
baud_rate = 115200

# シリアル通信の設定
ser = serial.Serial(com_port, baud_rate)

for i in range(500):
    # 現在の時刻を取得
    current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S\n")

    # 送信
    ser.write(current_time.encode("utf-8"))
    print(current_time)

    time.sleep(1)

ser.close()
