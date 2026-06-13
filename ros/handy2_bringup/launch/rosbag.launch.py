from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from datetime import datetime
import os

def generate_launch_description():
    # Generate timestamped bag name
    
    qos_config_path = os.path.join(
        get_package_share_directory('handy2_bringup'),
        'config',
        'rosbag_qos.yaml'
    )

    home_directory="/home/cub/"
    ext_storage_dir = os.path.join(home_directory, "ext_storage")
    rosbag_dir = os.path.join(home_directory, "rosbag")

    # リンク切れではなく、実体が存在する有効なディレクトリかを確認する
    if os.path.exists(ext_storage_dir) and os.path.isdir(ext_storage_dir):
        save_dir = ext_storage_dir
    else:
        save_dir = rosbag_dir
    bag_name = os.path.join(save_dir, 'bag_' + datetime.now().strftime('%Y_%m_%d-%H_%M_%S'))
    print(bag_name)
    
    # List of topics to record for handy2
    topic_list = [
        "/camera_node/camera_info",
        # "/camera_node/image_raw",
        "/camera_node/image_raw/compressed",
        # "/diagnostics",
        "/lidar_imu",
        # "/lidar_packets_loss",
        "/lidar_points",
        # "/parameter_events",
        # "/rosout",
    ]
    
    return LaunchDescription([
        Node(
            package='rosbag2_transport',
            executable='recorder',
            name='recorder',
            parameters=[{
                'storage.uri': bag_name,
                'storage.storage_id': 'mcap',
                'storage.max_cache_size': 1073741824, # 1GB
                'record.topics': topic_list,
                'record.qos_overrides_path': qos_config_path,
                'record.start_paused': False,
            }],
        ),
    ])
    #     return LaunchDescription([
    #     LoadComposableNodes(
    #         target_container='handy2_driver_container',
    #         composable_node_descriptions=[
    #             ComposableNode(
    #                 package='rosbag2_transport',
    #                 plugin='rosbag2_transport::Recorder',
    #                 name='recorder',
    #                 parameters=[{
    #                     'storage.uri': bag_name,
    #                     'storage.storage_id': 'mcap',
    #                     'storage.max_cache_size': 268435456,
    #                     'record.topics': topic_list,
    #                     'record.qos_overrides_path': qos_config_path,
    #                     'record.start_paused': False,
    #                     'use_intra_process_comms': True,
    #                 }],
    #             ),
    #         ],
    #     )
    # ])

