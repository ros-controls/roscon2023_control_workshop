# Copyright (c) 2021-2023 Stogl Robotics Consulting UG (haftungsbeschränkt)
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
#
# Author: Dr. Denis Stogl

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    hercules_1_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                FindPackageShare("hercules_description"),
                "/launch",
                "/hercules_sim_control.launch.py",
            ]
        ),
        launch_arguments={
            "prefix": "hercules_1/",
            "controller_manager_namespace": "hercules_1",
            "controller_manager_name": "/hercules_1/controller_manager",
            "controllers_file": "hercules_1_controllers.yaml",
            "launch_rviz": "false",
            "map_to_odom_y_axis": "-1",
            "odometry_frame_name": "hercules_1/odom",
        }.items(),
    )

    hercules_2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                FindPackageShare("hercules_description"),
                "/launch",
                "/hercules_sim_control.launch.py",
            ]
        ),
        launch_arguments={
            "prefix": "hercules_2/",
            "controller_manager_namespace": "hercules_2",
            "controller_manager_name": "/hercules_2/controller_manager",
            "controllers_file": "hercules_2_controllers.yaml",
            "launch_rviz": "true",
            "map_to_odom_y_axis": "1",
            "odometry_frame_name": "hercules_2/odom",
        }.items(),
    )

    nodes_to_start = [
        hercules_1_launch,
        hercules_2_launch,
    ]

    # set_odom_parameter = ExecuteProcess(
    #     cmd=[
    #         FindExecutable(name="ros2"),
    #         " param set ",
    #         "/hercules_1/set_scaling_factor ",
    #         "iras_interfaces/srv/SetScalingFactor ",
    #         '"{scaling_factor: 1.0}"',
    #     ],
    #     shell=True,
    # )

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
