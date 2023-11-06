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
    DeclareLaunchArgument,
    OpaqueFunction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# For port/starboard nomenclature see: https://en.wikipedia.org/wiki/Port_and_starboard


def launch_setup(context, *args, **kwargs):
    # General arguments
    start_arm_controllers = LaunchConfiguration("start_arm_controllers")
    base_controller = LaunchConfiguration("base_controller")
    port_joint_controller = LaunchConfiguration("port_joint_controller")
    port_gripper_controller = LaunchConfiguration("port_gripper_controller")
    starboard_joint_controller = LaunchConfiguration("starboard_joint_controller")
    starboard_gripper_controller = LaunchConfiguration("starboard_gripper_controller")

    controller_manager_name = LaunchConfiguration("controller_manager_name")

    # Base
    spawn_base_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[base_controller, "-c", controller_manager_name],
        output="screen",
    )

    # Port

    # There may be other controllers of the joints, but this is the initially-started one
    port_joint_controller_spawner_started = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            port_joint_controller,
            "-c",
            controller_manager_name,
        ],
        condition=IfCondition(start_arm_controllers),
    )
    port_joint_controller_spawner_stopped = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            port_joint_controller,
            "-c",
            controller_manager_name,
            "--stopped",
        ],
        condition=UnlessCondition(start_arm_controllers),
    )
    port_gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[port_gripper_controller, "-c", controller_manager_name],
    )

    # Starboard

    # There may be other controllers of the joints, but this is the initially-started one
    starboard_joint_controller_spawner_started = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            starboard_joint_controller,
            "-c",
            controller_manager_name,
        ],
        condition=IfCondition(start_arm_controllers),
    )
    starboard_joint_controller_spawner_stopped = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            starboard_joint_controller,
            "-c",
            controller_manager_name,
            "--stopped",
        ],
        condition=UnlessCondition(start_arm_controllers),
    )
    starboard_gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            starboard_gripper_controller,
            "-c",
            controller_manager_name,
        ],
    )

    nodes_to_start = [
        spawn_base_controller,
        port_joint_controller_spawner_stopped,
        port_joint_controller_spawner_started,
        port_gripper_controller_spawner,
        starboard_joint_controller_spawner_stopped,
        starboard_joint_controller_spawner_started,
        starboard_gripper_controller_spawner,
    ]

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "controller_manager_name",
            description="Name of the used controller manager.",
            default_value="/controller_manager",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "start_arm_controllers",
            default_value="true",
            description="Start arom controllers",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "base_controller",
            default_value="base_velocity_controller",
            description="Base controller to start.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "port_joint_controller",
            default_value="port_joint_trajectory_controller",
            description="Robot controller to start.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "starboard_joint_controller",
            default_value="starboard_joint_trajectory_controller",
            description="Robot controller to start.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "port_gripper_controller",
            default_value="port_gripper_controller",
            description="Gripper controller to start.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "starboard_gripper_controller",
            default_value="starboard_gripper_controller",
            description="Gripper controller to start.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz?")
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
