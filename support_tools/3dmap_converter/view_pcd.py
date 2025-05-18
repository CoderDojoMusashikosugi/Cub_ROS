import open3d as o3d

# 点群データを読み込む
pcd = o3d.io.read_point_cloud("no_ground.pcd")

# 点群データを表示
o3d.visualization.draw_geometries([pcd])

