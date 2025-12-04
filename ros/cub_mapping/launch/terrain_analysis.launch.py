"""
Launch file for terrain analysis node (2D obstacle detection from PCD files)

This node processes point cloud data from PCD files and generates
OccupancyGrid maps with obstacle detection based on terrain analysis.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from pathlib import Path


def generate_launch_description():
    # Package path
    pkg_share = FindPackageShare("cub_mapping").find("cub_mapping")
    param_file = Path(pkg_share) / "params" / "terrain_analysis.yaml"

    return LaunchDescription(
        [
            Node(
                package="cub_mapping",
                executable="terrain_analysis_node",
                name="terrain_analysis_node",
                output="screen",
                parameters=[str(param_file)],
                remappings=[
                    ("/cloud_pcd", "/cloud_pcd_mapping"),
                    ("/terrain_occupancy_grid", "/terrain_occupancy_grid"),
                    ("/ground_cloud", "/ground_cloud"),
                    ("/obstacle_cloud", "/obstacle_cloud"),
                ],
            ),
        ]
    )
