#!/usr/bin/env python3
# Copyright 2026 CoderDojo Musashikosugi / Cub_ROS
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Launch arguments that can override values in the YAML file.
# If an argument is left empty (default), the value in the YAML file is used.
OVERRIDABLE_PARAMS = {
    'port': 'Serial port device path (e.g. /dev/ttyUSB0)',
    'baudrate': 'Serial baudrate (e.g. 115200)',
    'enable_ntrip': 'Whether to enable NTRIP RTK corrections (true/false)',
    'frame_id': 'Primary antenna frame ID',
    'secondary_frame_id': 'Secondary antenna frame ID',
    'publish_secondary_fix': 'Whether to publish secondary antenna fix (true/false)',
}

# Parameter types (string params must stay strings, e.g. a frame_id like "1")
PARAM_CONVERTERS = {
    'port': str,
    'baudrate': int,
    'enable_ntrip': lambda v: v.strip().lower() in ('true', '1', 'yes', 'on'),
    'frame_id': str,
    'secondary_frame_id': str,
    'publish_secondary_fix': lambda v: v.strip().lower() in ('true', '1', 'yes', 'on'),
}


def _launch_setup(context, *args, **kwargs):
    config_file = LaunchConfiguration('config_file').perform(context)

    # Collect only explicitly specified overrides, converted to proper types.
    overrides = {}
    for name in OVERRIDABLE_PARAMS:
        value = LaunchConfiguration(name).perform(context)
        if value != '':
            overrides[name] = PARAM_CONVERTERS[name](value)

    parameters = [config_file]
    if overrides:
        parameters.append(overrides)

    um982_node = Node(
        package='cub_um982',
        executable='um982_node',
        name='um982_node',
        output='screen',
        parameters=parameters,
    )
    return [um982_node]


def generate_launch_description():
    pkg_share = get_package_share_directory('cub_um982')
    default_config_path = os.path.join(pkg_share, 'config', 'um982.yaml')

    # Declare launch arguments
    launch_args = [
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config_path,
            description='Path to the ROS2 parameter YAML file',
        ),
    ]
    for name, description in OVERRIDABLE_PARAMS.items():
        launch_args.append(DeclareLaunchArgument(
            name,
            default_value='',
            description=f'{description}. Empty = use value in config_file',
        ))

    return LaunchDescription(launch_args + [OpaqueFunction(function=_launch_setup)])
