from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction
from launch_ros.actions import PushRosNamespace
from launch.substitutions import LaunchConfiguration
import ament_index_python.packages
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
import yaml
import os

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

        # velodyneの起動
    velodyne_driver_share_dir = ament_index_python.packages.get_package_share_directory('velodyne_driver')
    _velodyne_driver_params_file = os.path.join(velodyne_driver_share_dir, 'config', 'VLP32C-velodyne_driver_node-params.yaml')
    velodyne_driver_node = launch_ros.actions.Node(package='velodyne_driver',
                                                   executable='velodyne_driver_node',
                                                   output='both',
                                                   parameters=[_velodyne_driver_params_file])
    velodyne_convert_share_dir = ament_index_python.packages.get_package_share_directory('velodyne_pointcloud')
    cub3_bringup_share_dir = ament_index_python.packages.get_package_share_directory('cub3_bringup')
    velodyne_convert_params_file = os.path.join(cub3_bringup_share_dir, 'config', 'VLP32C-velodyne_transform_node-params.yaml')
    with open(velodyne_convert_params_file, 'r') as f:
        velodyne_convert_params = yaml.safe_load(f)['velodyne_transform_node']['ros__parameters']
    velodyne_convert_params['calibration'] = os.path.join(velodyne_convert_share_dir, 'params', 'VeloView-VLP-32C.yaml')
    velodyne_transform_node = launch_ros.actions.Node(package='velodyne_pointcloud',
                                                      executable='velodyne_transform_node',
                                                      output='both',
                                                      parameters=[velodyne_convert_params])


    joy_dev = "/dev/input/js0"

    # Launchファイルの返り値
    return LaunchDescription([
        # pointcloud to laser scan
        Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            remappings=[('cloud_in', '/velodyne_points'),
                        ('scan', '/scan')],
            parameters=[{
                'target_frame': '',
                'transform_tolerance': 0.01,
                'min_height': -0.15,
                'max_height': 1.5,
                'angle_min': -3.14,  # -M_PI
                'angle_max': 3.14,  # M_PI
                'angle_increment': 0.0087,  # M_PI/360.0
                'scan_time': 0.3333,
                'range_min': 0.1,
                'range_max': 100.0,
                'use_inf': True,
                'inf_epsilon': 1.0
            }],
            name='pointcloud_to_laserscan'
        ),
               

        # 3D LiDAR -> common.launch.pyから移動
        velodyne_driver_node,
        velodyne_transform_node,

        Node(
            package='cub_commander',
            executable='cub_commander_node',
            output='screen',
            parameters=[{'dev': joy_dev}],
        ),

        Node(
            package='joy_linux',
            executable='joy_linux_node',
            parameters=[{'dev': joy_dev}],
        ),
    ])
