from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, GroupAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction
from launch_ros.actions import PushRosNamespace
import ament_index_python.packages
import launch_ros.actions
import yaml
import os

def generate_launch_description():
    micro_ros_agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        arguments=["serial", "--dev", "/dev/ttyATOM", "-b", "115200", "-v6"]
    )

    spresense_imu = Node(
        package='cxd5602pwbimu_localizer_node',
        executable='localizer_node',
        name='imu_localizer_node',
        parameters=[{
            'serial_port': "/dev/ttyMULIMU",
            'baud_rate': 1152000,
        }],
        output='screen'
    )

    joy_dev = "/dev/input/js0"
    cub_commander = Node(
        package='cub_commander',
        executable='cub_commander_node',
        output='screen',
        parameters=[{'dev': joy_dev}],
    )

    joy_linux = Node(
        package='joy_linux',
        executable='joy_linux_node',
        parameters=[{'dev': joy_dev}],
    )

    # sllidar_ros2パッケージの共有ディレクトリを取得
    sllidar_ros2_share_dir = FindPackageShare('sllidar_ros2').find('sllidar_ros2')
    
    # velodyneの起動
    velodyne_driver_share_dir = ament_index_python.packages.get_package_share_directory('velodyne_driver')
    _velodyne_driver_params_file = os.path.join(velodyne_driver_share_dir, 'config', 'VLP32C-velodyne_driver_node-params.yaml')
    velodyne_driver_node = launch_ros.actions.Node(package='velodyne_driver',
                                                   executable='velodyne_driver_node',
                                                   output='both',
                                                   parameters=[_velodyne_driver_params_file])
    velodyne_convert_share_dir = ament_index_python.packages.get_package_share_directory('velodyne_pointcloud')
    velodyne_convert_params_file = os.path.join(velodyne_convert_share_dir, 'config', 'VLP32C-velodyne_transform_node-params.yaml')
    with open(velodyne_convert_params_file, 'r') as f:
        velodyne_convert_params = yaml.safe_load(f)['velodyne_transform_node']['ros__parameters']
    velodyne_convert_params['calibration'] = os.path.join(velodyne_convert_share_dir, 'params', 'VeloView-VLP-32C.yaml')
    velodyne_transform_node = launch_ros.actions.Node(package='velodyne_pointcloud',
                                                      executable='velodyne_transform_node',
                                                      output='both',
                                                      parameters=[velodyne_convert_params])


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
    
    cub_bringup_params_path = os.path.join(get_package_share_directory('cub3_bringup'),'config')
    zed_f9p_params = os.path.join(cub_bringup_params_path, 'zed_f9p.yaml')
    rtklib = ExecuteProcess(
        cmd=['str2str', '-in','ntrip://ntrip1.bizstation.jp:2101/3041F3CA','-out','serial://ttyGPS:230400'],  #Tsukuba
        # cmd=['str2str', '-in','ntrip://ntrip1.bizstation.jp:2101/B4A00B46','-out','serial://ttyGPS:230400'],    #Yokohama
        output='both',
        respawn=True,
        respawn_delay=10.0
    )
    ublox_gps_delayed = TimerAction( # str2strをublox_gpsより先に起動しておく必要がある。
        period=5.0,
        actions=[
            Node(package='ublox_gps',
                executable='ublox_gps_node',
                output='both',
                parameters=[zed_f9p_params],
                respawn=True,
                respawn_delay=5.0
            ),
        ]
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
        # タイヤ
        micro_ros_agent,
        
        # spresense IMUノード
        spresense_imu,
        
        # 手動操縦
        cub_commander,        
        joy_linux,

        # 2D LiDAR
        slc_L_group,
        slc_R_group,

        # GNSS
        rtklib,
        ublox_gps_delayed,

        # 3D LiDAR -> commmon.launch.pyでの起動に移動
        # velodyne_driver_node,
        # velodyne_transform_node,

        # RGB-D Camera -> commmon.launch.pyでの起動に移動
        # realsense_launch,
    ])
