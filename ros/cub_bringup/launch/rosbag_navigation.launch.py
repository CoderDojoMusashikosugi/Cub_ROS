from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch.actions import ExecuteProcess
from launch.actions import TimerAction
from launch.actions import LogInfo
import os

def generate_launch_description():
    cub_target = os.getenv('CUB_TARGET', 'cub4')
    print("launch target:", cub_target)

    home_directory="/home/cub/"
    ext_storage_dir = os.path.join(home_directory, "ext_storage")
    rosbag_dir = os.path.join(home_directory, "rosbag")

    # リンク切れではなく、実体が存在する有効なディレクトリかを確認する
    if os.path.exists(ext_storage_dir) and os.path.isdir(ext_storage_dir):
        save_dir = ext_storage_dir
    else:
        save_dir = rosbag_dir
    topic_list=[
        # "/behavior_server/transition_event",
        # "/behavior_tree_log",
        # "/bond",
        # "/bt_navigator/transition_event",
        # "/clicked_point",
        "/cmd_vel", #要る
        "/cmd_vel_atom", #要る
        "/cmd_vel_nav", #要る
        "/cmd_vel_smoothed", #要る
        "/cmd_vel_teleop", #要る
        "/collision_monitor/collision_points_marker",
        # "/collision_monitor/transition_event",
        "/collision_monitor_state",
        # "/controller_server/transition_event",
        # "/diag_overlay",
        "/diagnostics",
        "/diagnostics_agg",
        "/diagnostics_toplevel_state",
        "/dial/button",
        # "/downsampled_costmap", # いらなさすぎる
        # "/downsampled_costmap_updates", # いらなさすぎる
        "/follow_mode_enabled",
        "/global_costmap/costmap", #要る
        # "/global_costmap/costmap_raw",
        # "/global_costmap/costmap_raw_updates",
        # "/global_costmap/costmap_updates",
        # "/global_costmap/footprint",
        # "/global_costmap/global_costmap/transition_event",
        # "/global_costmap/obstacle_layer",
        # "/global_costmap/obstacle_layer_raw",
        # "/global_costmap/obstacle_layer_raw_updates",
        # "/global_costmap/obstacle_layer_updates",
        "/global_costmap/published_footprint",
        # "/global_costmap/static_layer",
        # "/global_costmap/static_layer_raw",
        # "/global_costmap/static_layer_raw_updates",
        # "/global_costmap/static_layer_updates",
        # "/global_costmap/voxel_marked_cloud",
        "/goal_pose",
        "/goal_update",
        "/gps_pose",
        "/imu/data",
        "/imu/data_raw", #要る
        "/imu/temperature", #要る
        "/initialpose", #要る
        "/joint_states", #要る
        "/joy", #要る
        "/joy/set_feedback", #要る
        "/livox/livox/imu", #要る
        "/livox/livox/lidar", #要る
        "/livox/livox/time_type", #要る
        "/local_costmap/costmap", #要る
        # "/local_costmap/costmap_raw",
        # "/local_costmap/costmap_raw_updates",
        # "/local_costmap/costmap_updates",
        "/local_costmap/footprint",
        # "/local_costmap/local_costmap/transition_event",
        # "/local_costmap/obstacle_layer",
        # "/local_costmap/obstacle_layer_raw",
        # "/local_costmap/obstacle_layer_raw_updates",
        # "/local_costmap/obstacle_layer_updates",
        # "/local_costmap/published_footprint",
        # "/local_costmap/voxel_marked_cloud",
        # "/local_map",
        "/map",  #要る
        # "/map_out",
        "/map_server/transition_event",  #要る
        "/map_updates",  #要る
        # "/mobile_base/sensors/bumper_pointcloud",
        # "/move_base_simple/goal", # いらなさすぎる
        # "/mros_debug_topic",
        # "/ndt_pc",
        "/odom", #要る
        "/optimal_trajectory", #要る
        "/parameter_events",
        # "/particle_cloud", #　いらなさすぎる
        "/plan", #要る
        "/plan_smoothed", #要る
        # "/planner_server/transition_event",
        "/preempt_teleop",
        "/proceed_to_next_group", #要る
        "/robot_description", #要る
        "/rosout", #要る
        # "/route_graph", #なにこれ
        # "/route_server/transition_event",
        # "/scan", #変換で再現可能かなと思って外してる
        # "/scan_livox",
        # "/smoother_server/transition_event",
        "/speed_limit", #要る
        "/test/ekf_pose", #要る
        "/test/ndt_pose", #要る
        "/tf", #要る
        "/tf_static", #要る
        "/trajectories", #要る
        "/transformed_global_plan", #要る
        "/um982_node/fix", #要る
        "/um982_node/fix_secondary", #要る
        "/um982_node/pose", #要る
        "/velocity_polygon", #要る
        "/velocity_polygon_slow", #要る
        # "/velocity_smoother/transition_event",
        # "/velodyne_packets",
        "/velodyne_points",#要る
        # "/waypoint_follower/transition_event",
        "/waypoints", #要る
        "/wheel_positions", #要る
        "/wp_vis",
        "/wp_vis_array", #要る
    ]
    return LaunchDescription([
        # IncludeLaunchDescription(
        #     PathJoinSubstitution(
        #         [FindPackageShare("cub_bringup"), "launch", "common.launch.py"]
        #     ),
        # ),

        LogInfo(msg=f"rosbag is saving to: {save_dir}"),

        TimerAction(period=3.0, actions=[ExecuteProcess(
            cwd=save_dir,
            cmd=['ros2', 'bag', 'record', '-s', 'mcap'] + topic_list,
            output='screen'
        )])
    ])
