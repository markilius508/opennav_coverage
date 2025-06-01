# Copyright (c) 2023 Open Navigation LLC
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
from launch.actions import ExecuteProcess, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    coverage_demo_dir = get_package_share_directory('opennav_coverage_demo')

    param_file_path = os.path.join(coverage_demo_dir, 'gps_full_params.yaml')
    amcl_config_path = os.path.join(coverage_demo_dir, 'amcl.yaml')

    map_name = LaunchConfiguration("map_name")
    use_sim_time = LaunchConfiguration("use_sim_time")
    amcl_config = LaunchConfiguration("amcl_config")

    map_name_arg = DeclareLaunchArgument(
        "map_name"
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value = "true"
    )

    amcl_config_arg = DeclareLaunchArgument(
        "amcl_config",
        default_value = amcl_config_path,
        description = "Full path to amcl yaml file to load"
    )

    # start the simulation
    gazebo_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([coverage_demo_dir, '/turtlebot3_house.launch.py']))
    
    # world->odom transform, no localization. For visualization & controller transform
    amcl_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([coverage_demo_dir, '/amcl.launch.py']),
            launch_arguments={'map_name': map_name,
                              'use_sim_time': use_sim_time,
                              'amcl_config': amcl_config,
                              'container_name': 'nav2_container'}.items()
            )

    # start the visualization
    rviz_config = os.path.join(coverage_demo_dir, 'opennav_coverage_demo.rviz')
    # print("rviz_config:", rviz_config)
    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'rviz_launch.py')),
        launch_arguments={'namespace': '', 'rviz_config': rviz_config}.items())

    # start navigation
    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(coverage_demo_dir, 'bringup_full_launch.py')),
        launch_arguments={'params_file': param_file_path}.items())

    # start the demo task
    demo_cmd = Node(
        package='opennav_coverage_demo',
        executable='gps_raceway_coverage',
        emulate_tty=True,
        output='screen')

    ld = LaunchDescription()
    ld.add_action(map_name_arg)
    ld.add_action(use_sim_time_arg)
    ld.add_action(amcl_config_arg)
    ld.add_action(gazebo_launch)
    ld.add_action(amcl_launch)
    ld.add_action(rviz_cmd)
    ld.add_action(bringup_cmd)
    ld.add_action(demo_cmd)
    return ld
