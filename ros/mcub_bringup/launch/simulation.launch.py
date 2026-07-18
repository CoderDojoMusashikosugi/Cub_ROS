from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    xacro_path = PathJoinSubstitution([
        FindPackageShare('mcub_bringup'),
        'urdf',
        'mcub.urdf.xacro',
    ])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {
                    'robot_description': ParameterValue(
                        Command(['xacro ', xacro_path]),
                        value_type=str,
                    ),
                },
                {'use_sim_time': use_sim_time},
            ],
        ),
    ])
