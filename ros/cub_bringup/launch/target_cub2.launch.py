from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # パラメータの宣言
    L_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    frame_id = LaunchConfiguration('frame_id', default='laser_frame')
    inverted = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')

    # sllidar_ros2パッケージの共有ディレクトリを取得
    sllidar_ros2_share_dir = get_package_share_directory('sllidar_ros2')
    
    #sllidarの起動　LとRで分ける。
    sllidar_L_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            sllidar_ros2_share_dir,
            '/launch/sllidar_c1_launch.py'
        ]),
        launch_arguments={
            'serial_port': serial_port,
            'frame_id': frame_id,
            'inverted': inverted,
            'angle_compensate': angle_compensate
        }.items()
    )
    sllidar_R_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            sllidar_ros2_share_dir,
            '/launch/sllidar_c1_launch.py'
        ]),
        launch_arguments={
            'serial_port': serial_port,
            'frame_id': frame_id,
            'inverted': inverted,
            'angle_compensate': angle_compensate
        }.items()
    )
    
    
    return LaunchDescription([
        Node(
            package='wheel_odometry',
            executable='wheel_odometry_node',
        ),

        IncludeLaunchDescription(
            PathJoinSubstitution(
                [FindPackageShare("cub_visualization"), "launch", "rviz.launch.py"]
            ),
        ),
        Node(
            packages='bno055'
            executable='bno055'
            name=''
            arguments=['--params-file',"/home/musashikosugi/colcon_ws/src/bno055/bno055/params/bno055_params.yaml"]
        ),
        sllidar_L_launch,
        sllidar_R_launch
    ])