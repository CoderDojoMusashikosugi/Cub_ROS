import open3d as o3d

# 点群データを読み込む
pcd = o3d.io.read_point_cloud("0.15_map_all.pcd")

# 点群を処理する (例: ダウンサンプリングや平面除去)
plane_model, inliers = pcd.segment_plane(distance_threshold=2.00,
                                         ransac_n=3,
                                         num_iterations=1000)
# 路面以外の点群を取得
non_ground_pcd = pcd.select_by_index(inliers, invert=True)

# 結果を保存
o3d.io.write_point_cloud("no_ground.pcd", non_ground_pcd, write_ascii=True)

