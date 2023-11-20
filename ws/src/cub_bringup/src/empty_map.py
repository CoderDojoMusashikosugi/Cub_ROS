#!/usr/bin/env python3

import rospy
from nav_msgs.msg import OccupancyGrid

def create_empty_occupancy_grid():
    # ROSノードの初期化
    rospy.init_node('empty_occupancy_grid_node', anonymous=True)

    # OccupancyGridメッセージの作成
    occupancy_grid = OccupancyGrid()

    # メッセージのヘッダー情報の設定
    occupancy_grid.header.stamp = rospy.Time.now()
    occupancy_grid.header.frame_id = "map"

    # グリッドのサイズと解像度の設定
    occupancy_grid.info.width = 400  # グリッドの幅（セル数）
    occupancy_grid.info.height = 400  # グリッドの高さ（セル数）
    occupancy_grid.info.resolution = 1.0  # グリッドの解像度（メートル/セル）

    # グリッドの原点の座標の設定
    occupancy_grid.info.origin.position.x = -200.0  # 左上のセルのx座標（メートル）
    occupancy_grid.info.origin.position.y = -200.0  # 左上のセルのy座標（メートル）
    occupancy_grid.info.origin.position.z = 0.0

    # メッセージのデータ（オクルージョンデータ）を初期化
    # 未知の領域は-1、空白は0、占有されている領域は100で表現
    occupancy_grid.data = [0] * (occupancy_grid.info.width * occupancy_grid.info.height)

    # OccupancyGridメッセージを配信
    occupancy_grid_pub = rospy.Publisher('empty_occupancy_grid', OccupancyGrid, queue_size=10)


    # 1秒ごとにメッセージを送信するためのループ
    rate = rospy.Rate(1)  # 1Hz
    while not rospy.is_shutdown():
        occupancy_grid.header.stamp = rospy.Time.now()
        occupancy_grid_pub.publish(occupancy_grid)
        rate.sleep()
    # rospy.spin()

if __name__ == '__main__':
    try:
        create_empty_occupancy_grid()
    except rospy.ROSInterruptException:
        pass