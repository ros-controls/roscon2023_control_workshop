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
#
# Description: After a robot has been loaded, this will execute a series of trajectories.

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    position_goals = PathJoinSubstitution(
        [
            FindPackageShare("hercules_description"),
            "config",
            "test_multi_dual_goal_publishers_config.yaml",
        ]
    )

    return LaunchDescription(
        [
            Node(
                package="ros2_controllers_test_nodes",
                executable="publisher_joint_trajectory_controller",
                name="hercules_1_publisher_port_joint_trajectory_controller",
                parameters=[position_goals],
                output={
                    "stdout": "screen",
                    "stderr": "screen",
                },
            ),
            Node(
                package="ros2_controllers_test_nodes",
                executable="publisher_joint_trajectory_controller",
                name="hercules_1_publisher_starboard_joint_trajectory_controller",
                parameters=[position_goals],
                output={
                    "stdout": "screen",
                    "stderr": "screen",
                },
            ),
            Node(
                package="ros2_controllers_test_nodes",
                executable="publisher_forward_position_controller",
                name="hercules_1_publisher_port_gripper_controller",
                parameters=[position_goals],
                output={
                    "stdout": "screen",
                    "stderr": "screen",
                },
            ),
            Node(
                package="ros2_controllers_test_nodes",
                executable="publisher_forward_position_controller",
                name="hercules_1_publisher_starboard_gripper_controller",
                parameters=[position_goals],
                output={
                    "stdout": "screen",
                    "stderr": "screen",
                },
            ),
            ExecuteProcess(
                cmd=[
                    "ros2 topic pub  -r 10 /hercules_1_base_velocity_controller/cmd_vel_unstamped geometry_msgs/msg/Twist '{linear: {x: 0.4, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.2}}'"
                ],
                shell=True,
            ),
            Node(
                package="ros2_controllers_test_nodes",
                executable="publisher_joint_trajectory_controller",
                name="hercules_2_publisher_port_joint_trajectory_controller",
                parameters=[position_goals],
                output={
                    "stdout": "screen",
                    "stderr": "screen",
                },
            ),
            Node(
                package="ros2_controllers_test_nodes",
                executable="publisher_joint_trajectory_controller",
                name="hercules_2_publisher_starboard_joint_trajectory_controller",
                parameters=[position_goals],
                output={
                    "stdout": "screen",
                    "stderr": "screen",
                },
            ),
            Node(
                package="ros2_controllers_test_nodes",
                executable="publisher_forward_position_controller",
                name="hercules_2_publisher_port_gripper_controller",
                parameters=[position_goals],
                output={
                    "stdout": "screen",
                    "stderr": "screen",
                },
            ),
            Node(
                package="ros2_controllers_test_nodes",
                executable="publisher_forward_position_controller",
                name="hercules_2_publisher_starboard_gripper_controller",
                parameters=[position_goals],
                output={
                    "stdout": "screen",
                    "stderr": "screen",
                },
            ),
            ExecuteProcess(
                cmd=[
                    "ros2 topic pub  -r 10 /hercules_2_base_velocity_controller/cmd_vel_unstamped geometry_msgs/msg/Twist '{linear: {x: 0.4, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.2}}'"
                ],
                shell=True,
            ),
        ]
    )
