# Copyright 2022 Open Source Robotics Foundation, Inc.
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
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node


def generate_launch_description():
    # Configure ROS nodes for launch

    # Setup project paths
    pkg_ros_slam_toolbox = get_package_share_directory('slam_toolbox')
    pkg_project_skid_steer = get_package_share_directory('ros_gz_example_bringup')

   
    # Launch the skid_steer lauch file, this will open gazebo, rviz, etc
    skid_steer_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_project_skid_steer, 'launch', 'skid_steer.launch.py')),
        launch_arguments={
        }.items(),
    )

    # Launch the SLAM toolbox
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_slam_toolbox, 'launch', 'online_async_launch.py')),
        launch_arguments={'slam_params_file': PathJoinSubstitution([
            pkg_ros_slam_toolbox,
            'config',
            'mapper_params_online_async.yaml'
        ]),
        'use_sim_time': 'true'
        }.items(),
    )

    # Issues arise with putting this in launch file because the model already needs to be running
    # Thus best to put it in an interactive terminal or its own launch file

    # Replacement code:
    '''
    ros2 run teleop_twist_keyboard teleop_twist_keyboard /cmd_vel:=/skid_steer/cmd_vel
    '''

    # teleop_keyboard = Node(
    #     package='teleop_twist_keyboard',
    #     executable='teleop_twist_keyboard',
    #     remappings=[('/cmd_vel', '/skid_steer/cmd_vel')],
    #     output='screen'
    # )

    return LaunchDescription([
        skid_steer_sim,
        slam_launch,
    ])
