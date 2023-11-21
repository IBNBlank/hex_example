#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2023 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2023-11-21
################################################################

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    hex_example_param = os.path.join(
        get_package_share_directory("hex_example"), "config/ros2",
        "hex_example.yaml")

    hex_example = Node(name="hex_example",
                       package="hex_example",
                       executable="hex_example",
                       output="screen",
                       parameters=[hex_example_param, {
                           "max_count": 20
                       }],
                       remappings=[("/in_string", "/in"),
                                   ("/out_string", "/out")])

    return LaunchDescription([hex_example])
