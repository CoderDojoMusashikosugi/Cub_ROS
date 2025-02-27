from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction
from launch_ros.actions import PushRosNamespace
import os

def generate_launch_description():
    # sllidar_ros2パッケージの共有ディレクトリを取得
    sllidar_ros2_share_dir = FindPackageShare('sllidar_ros2').find('sllidar_ros2')
    
    velodyne_launch_file_dir = os.path.join(
        get_package_share_directory('velodyne'),
        'launch',
        'velodyne-all-nodes-VLP32C-launch.py'
    )

    realsense_launch_file_dir = os.path.join(
        get_package_share_directory("realsense2_camera"),
        'launch'
    )
    
    # sllidarの起動　LとRで分ける
    sllidar_L_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(sllidar_ros2_share_dir, 'launch', 'sllidar_c1_launch.py')
        ]),
        launch_arguments={
            'serial_port': "/dev/ttySLC1L",
            'frame_id': "SLC1L",
            'topic_name': "/SLC1L_scan",
        }.items()
    )
    sllidar_L_launch_delayed = TimerAction(period=1.0, actions=[sllidar_L_launch])
    
    # R側の設定
    sllidar_R_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(sllidar_ros2_share_dir, 'launch', 'sllidar_c1_launch.py')
        ]),
        launch_arguments={
            'serial_port': "/dev/ttySLC1R",
            'frame_id': "SLC1R",
            'topic_name': "/SLC1R_scan",
        }.items()
    )
    sllidar_R_launch_delayed = TimerAction(period=3.0, actions=[sllidar_R_launch])
    
    # GPSのlaunchファイル
    # gps_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(
    #             FindPackageShare('nmea_navsat_driver').find('nmea_navsat_driver'),
    #             'launch',
    #             'nmea_serial_driver.launch.py'
    #         )
    #     )
    # )

    ublox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                FindPackageShare('ublox_gps').find('ublox_gps'),
                "launch",
                "ublox_gps_node_zedf9p-launch.py"
            )

        )
    )

    # velodyneの起動
    velodyne_launch=IncludeLaunchDescription(
            PythonLaunchDescriptionSource(velodyne_launch_file_dir)
        )
    
    realsense_launch=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(realsense_launch_file_dir, "rs_launch.py")),
        launch_arguments={
            'config_file': os.path.join(realsense_launch_file_dir, "config", "config.yaml"),
        }.items()
    )
    
    # LIDARのGroupAction
    slc_L_group = GroupAction(
        actions=[PushRosNamespace('sllidar_l'),sllidar_L_launch_delayed],
        scoped=True
    )

    slc_R_group = GroupAction(
        actions=[PushRosNamespace('sllidar_r'),sllidar_R_launch_delayed],
        scoped=True
    )
    
    # Launchファイルの返り値
    return LaunchDescription([
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            arguments=["serial", "--dev", "/dev/ttyATOM", "-b", "115200", "-v6"]
        ),

        # bno055ノード
        # serialerror以外は通常通り動きます
        Node(
            package='bno055',
            executable='bno055',
            name='bno055',
            parameters=[{
                'uart_port': "/dev/ttyBNO055"
            }],
            output='screen'
        ),

        # グループアクション
        slc_L_group,
        slc_R_group,

        # GPS launch
        #gps_launch,
        # velodyne launch
        velodyne_launch,
        realsense_launch,
    ])
