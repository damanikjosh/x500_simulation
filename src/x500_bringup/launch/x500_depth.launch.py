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

import xacro

def generate_launch_description():
    # Configure ROS nodes for launch

    # Setup project paths
    pkg_project_bringup = get_package_share_directory('x500_bringup')
    pkg_project_gazebo = get_package_share_directory('x500_gazebo')
    pkg_project_description = get_package_share_directory('x500_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Load the SDF file from "description" package
    world_file = os.path.join(pkg_project_gazebo, 'worlds', 'default.sdf')
    urdf_file  =  os.path.join(pkg_project_description, 'models', 'x500_depth', 'model.sdf.xacro')
    config_file = os.path.join(pkg_project_description, 'models', 'x500_depth', 'bridge.yaml')

    robot_desc = xacro.process_file(urdf_file).toprettyxml(indent='  ')


    # Setup to launch the simulator and Gazebo world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': world_file}.items(),
    )

    gz_spawn_model = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'ros_gz_spawn_model.launch.py')),
        launch_arguments={
            'bridge_name': 'ros_gz_bridge',
            'world': 'default',
            'config_file': config_file,
            'model_string': robot_desc,
            'x': '0.0',
            'y': '0.0',
            'z': '0.23',
        }.items(),
    )

    # Takes the description and joint angles as inputs and publishes the 3D poses of the robot links
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_desc},
        ]
    )

    # NavSat Transform Node
    # navsat_transform_node = Node(
    #     package='robot_localization',
    #     executable='navsat_transform_node',
    #     name='navsat_transform_node',
    #     output='screen',
    #     parameters=[{
    #         'use_sim_time': True,
    #         'frequency': 10.0
    #     }]
    # )


    # Visualize in RViz
    rviz = Node(
       package='rviz2',
       executable='rviz2',
       arguments=['-d', os.path.join(pkg_project_bringup, 'config', 'x500.rviz')],
       condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription([
        gz_sim,
        gz_spawn_model,
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Open RViz.'),
        robot_state_publisher,
        # navsat_transform_node,
        rviz
    ])
