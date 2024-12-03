import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # パラメータファイルのパスを取得
    params_ekf = os.path.join(
        get_package_share_directory('ekf_localizer'),  # パッケージ名
        'config',  # ディレクトリ名
        'ekf',  # サブディレクトリ
        'ekf.yaml'  # ファイル名
    )
    params_map_matcher = os.path.join(
        get_package_share_directory('ekf_localizer'),  # パッケージ名
        'config',  # ディレクトリ名
        'map_matcher',  # サブディレクトリ
        'map_matcher.yaml'  # ファイル名
    )
    pcd_params = {
        "file_name": "/home/cub/colcon_ws/src/cub/ekf_localizer/pcd/map_msakosu.pcd",
        "frame_id": "map",
        "publish_rate": 0.5
    }

    return LaunchDescription([
        # ノード1: EKF Localizer
        Node(
            package='ekf_localizer',
            executable='ekf_localizer_node',
            name='ekf_localizer_node',
            parameters=[params_ekf],
            # output='screen',  # 標準出力を表示するように設定
        ),
        
        # ノード2: Map Matcher
        Node(
            package='ekf_localizer',
            executable='map_matcher_node',
            name='map_matcher_node',
            parameters=[params_map_matcher],
            # output='screen',  # 標準出力を表示
        ),

        # ノード3: TF Cub
        Node(
            package='ekf_localizer',
            executable='tf_cub_node',
            name='tf_cub_node',
            # output='screen',  # 標準出力を表示
        ),
        # ↓なぜか/base_link座標系に出力されてしまい
        # Node(
        #     package='pcl_ros',
        #     executable='pcd_to_pointcloud',
        #     name='pcd_publisher',
        #     output='screen',  # ログをターミナルに表示
        #     parameters=[
        #         {"file_name": "/home/cub/colcon_ws/src/cub/ekf_localizer/pcd/map_msakosu.pcd"},  # 正しいパラメータ名
        #         {"frame_id": "map"},  # 座標フレーム名
        #         {"publish_rate": 0.5}  # パブリッシュ間隔（Hz）
        #     ]
        # )
    ])


