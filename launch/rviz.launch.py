#!/usr/bin/env python3

# Copyright 2020 ros2_control Development Team
# Copyright 2024 Husarion sp. z o.o.
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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import ReplaceString


def generate_launch_description():

    namespace = LaunchConfiguration("namespace")
    declare_namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value=EnvironmentVariable("ROBOT_NAMESPACE", default_value=""),
        description="Add namespace to all launched nodes.",
    )

    rviz_config = LaunchConfiguration("rviz_config")
    declare_rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=PathJoinSubstitution(
            [FindPackageShare("husarion_ugv_description"), "rviz", "husarion_ugv.rviz"]
        ),
        description="RViz configuration file.",
    )

    use_sim = LaunchConfiguration("use_sim")
    declare_use_sim_arg = DeclareLaunchArgument(
        "use_sim",
        default_value="False",
        description="Whether simulation is used.",
        choices=["True", "true", "False", "false"],
    )

    ns_ext = PythonExpression(["'", namespace, "' + '/' if '", namespace, "' else ''"])

    rviz_config = ReplaceString(
        source_file=rviz_config,
        replacements={"<namespace>/": ns_ext, "<namespace>": namespace},
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        namespace=namespace,
        arguments=["-d", rviz_config],
    )

    actions = [
        declare_namespace_arg,
        declare_rviz_config_arg,
        declare_use_sim_arg,
        SetParameter(name="use_sim_time", value=use_sim),
        rviz_node,
    ]

    return LaunchDescription(actions)
