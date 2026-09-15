#!/usr/bin/env python3
# Copyright 2026 ros2_control Development Team
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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('ros2_control_transport_demo')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': open(
            pkg + '/description/transport_demo.urdf').read()}],
    )

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': open(pkg + '/description/transport_demo.urdf').read()},
            pkg + '/config/transport_demo_controllers.yaml',
        ],
        output='screen',
    )

    spawn_jsb = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen',
    )

    return LaunchDescription([
        robot_state_publisher,
        controller_manager,
        RegisterEventHandler(
            OnProcessExit(target_action=controller_manager, on_exit=[spawn_jsb])),
    ])
