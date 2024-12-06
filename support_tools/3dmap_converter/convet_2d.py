import numpy as np
import cv2

# 点群データを読み込む
cloud = np.loadtxt('no_ground.pcd', skiprows=11)  # 必要に応じてスキップ行を調整

# グリッド解像度（1セルのサイズ、単位：メートル）
resolution = 0.05

# グリッドサイズを計算
x_min, x_max = cloud[:, 0].min(), cloud[:, 0].max()
y_min, y_max = cloud[:, 1].min(), cloud[:, 1].max()
width = int((x_max - x_min) / resolution) + 1
height = int((y_max - y_min) / resolution) + 1

# Occupancy Gridを初期化
grid = np.zeros((height, width), dtype=np.uint8)

# 点をグリッドにマッピング
for point in cloud:
    x_idx = int((point[0] - x_min) / resolution)
    y_idx = int((point[1] - y_min) / resolution)

    # 範囲外アクセスを防ぐ
    if 0 <= x_idx < width and 0 <= y_idx < height:
        grid[y_idx, x_idx] = 255  # 占有セル

# マップを保存
cv2.imwrite('map_tsukuba_net.pgm', grid)

