from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, GroupAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction
from launch_ros.actions import PushRosNamespace
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
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
        package='cub4_bringup',
        executable='spresense_imu_node',
        name='spresense_imu_node',
        parameters=[{
            'serial_port': "/dev/ttyMULIMU",
            'baud_rate': 230400,
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

    # cub4_bringup_launchディレクトリを取得
    cub_bringup_path = os.path.join(get_package_share_directory('cub4_bringup'))
    cub_bringup_launch_path = os.path.join(cub_bringup_path,'launch')
    
    # velodyneの起動
    # velodyne_driver_share_dir = ament_index_python.packages.get_package_share_directory('velodyne_driver')
    _velodyne_driver_params_file = os.path.join(cub_bringup_path, 'config', 'VLP32C-velodyne_driver_node-params.yaml')
    # velodyne_driver_node = launch_ros.actions.Node(package='velodyne_driver',
    #                                                executable='velodyne_driver_node',
    #                                                output='both',
    #                                                parameters=[_velodyne_driver_params_file])
    with open(_velodyne_driver_params_file, 'r') as f:
        params = yaml.safe_load(f)['velodyne_driver_node']['ros__parameters']
    container = ComposableNodeContainer(
            name='velodyne_driver_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='velodyne_driver',
                    plugin='velodyne_driver::VelodyneDriver',
                    name='velodyne_driver_node',
                    parameters=[params]),
            ],
            output='both',
    )
    velodyne_driver_node = LaunchDescription([container])

    velodyne_convert_share_dir = ament_index_python.packages.get_package_share_directory('velodyne_pointcloud')
    velodyne_convert_params_file = os.path.join(velodyne_convert_share_dir, 'config', 'VLP32C-velodyne_transform_node-params.yaml')
    with open(velodyne_convert_params_file, 'r') as f:
        velodyne_convert_params = yaml.safe_load(f)['velodyne_transform_node']['ros__parameters']
    velodyne_convert_params['calibration'] = os.path.join(velodyne_convert_share_dir, 'params', 'VeloView-VLP-32C.yaml')
    velodyne_transform_node = launch_ros.actions.Node(package='velodyne_pointcloud',
                                                      executable='velodyne_transform_node',
                                                      output='both',
                                                      parameters=[velodyne_convert_params])
    
    # livoxの起動
    livox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(cub_bringup_launch_path, 'msg_MID360_launch.py')
        ]),
    )
    livox_delayed = TimerAction(period=1.0, actions=[livox_launch])
    
    # UM982 GNSSの起動
    um982_params_file = os.path.join(cub_bringup_path, 'config', 'um982.yaml')
    um982 = Node(
        package='cub_um982',
        executable='um982_node',
        name='um982_node',
        output='screen',
        parameters = [um982_params_file],
    )

    # LIDARのGroupAction
    livox_group = GroupAction(
        actions=[PushRosNamespace('livox'),livox_delayed],
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

        # Livox
        livox_group,

        # GNSS
        um982,

        # 3D LiDAR -> commmon.launch.pyでの起動に移動
        velodyne_driver_node,
        velodyne_transform_node,
    ])
