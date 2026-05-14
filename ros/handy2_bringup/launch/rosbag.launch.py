from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from datetime import datetime
import os

def generate_launch_description():
    # Generate timestamped bag name
    bag_name = os.path.join('/home/cub/rosbag', 'bag_' + datetime.now().strftime('%Y_%m_%d-%H_%M_%S'))
    
    topic_list = [
        "/camera_node/camera_info",
        "/camera_node/image_raw/compressed",
    ]
    
    # Comma separated topic list
    topics_str = ",".join(topic_list)
    
    # Script path
    script_path = PathJoinSubstitution([
        FindPackageShare('handy2_bringup'),
        'scripts',
        'composable_recorder_manager.py'
    ])

    return LaunchDescription([
        ExecuteProcess(
            cmd=[script_path],
            output='screen',
            additional_env={
                'ROSBAG_NAME': bag_name,
                'ROSBAG_TOPICS': topics_str,
                'PYTHONUNBUFFERED': '1'
            }
        )
    ])
