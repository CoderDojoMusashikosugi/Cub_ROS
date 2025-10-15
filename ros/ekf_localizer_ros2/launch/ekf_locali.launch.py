import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import TimerAction

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
        "frame_id": "map",
        "publish_rate": 0.5
    }
    params_gps_updater = os.path.join(
        get_package_share_directory('ekf_localizer'),  # パッケージ名
        'config',  # ディレクトリ名
        'gps_updater',  # サブディレクトリ
        'gps_updater.yaml'  # ファイル名
    )

    return LaunchDescription([
        # IncludeLaunchDescription(
        #     PathJoinSubstitution(
        #         [FindPackageShare("cub_visualization"), "launch", "rviz.launch.py"]
        #     ),
        # ),
        # ノード1: EKF Localizer
        Node(
            package='ekf_localizer',
            executable='ekf_localizer_node',
            name='ekf_localizer_node',
            parameters=[params_ekf],
            remappings=[('/bno055/imu', '/imu/data'),
                                ],
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
        Node(
            package='ekf_localizer',
            executable='gps_updater_node',
            name='gps_updater',
            parameters=[params_gps_updater],
            # output='screen',
        ),

        # wheel_odometryノード
        # Node(
        #     package='wheel_odometry',
        #     executable='wheel_odometry_node',
        #     name='wheel_odometry_node',
        #     # output='screen'
        # ),
        # Node(
        #     package='pcl_ros',
        #     executable='pcd_to_pointcloud',
        #     name='pcd_publisher',
        #     output='screen',  # ログをターミナルに表示
        #     parameters=[
        #         {"file_name": "/home/cub/colcon_ws/src/cub/ekf_localizer/pcd/map_siminkaikan.pcd"},  # 正しいパラメータ名
        #         {"frame_id": "/map"},  # 座標フレーム名
        #         {"publish_rate": 0.5}  # パブリッシュ間隔（Hz）
        #     ],
        # ),
        # Node(
        #     package='ekf_localizer',
        #     executable='pcd_renamer',
        #     name='pcd_renamer',
        # ),
        # Node(
        #     package='ekf_localizer',
        #     executable='draw_moving_trajectory_node',
        #     name='draw_moving_trajectory_node',
        # ),
    ])


