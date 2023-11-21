#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2023 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2023-11-20
################################################################

import sys
if sys.version_info.major < 3:
    print("Use python3 to run this file!")
    exit()

import os
import subprocess

PKG_NAME = "hex_example"

COLORS = dict()
COLORS['BLACK'] = "\033[0;30m"
COLORS['RED'] = "\033[0;31m"
COLORS['GREEN'] = "\033[0;32m"
COLORS['BROWN'] = "\033[0;33m"
COLORS['BLUE'] = "\033[0;34m"
COLORS['PURPLE'] = "\033[0;35m"
COLORS['CYAN'] = "\033[0;36m"
COLORS['LIGHT_GRAY'] = "\033[0;37m"
COLORS['DARK_GRAY'] = "\033[1;30m"
COLORS['LIGHT_RED'] = "\033[1;31m"
COLORS['LIGHT_GREEN'] = "\033[1;32m"
COLORS['YELLOW'] = "\033[1;33m"
COLORS['LIGHT_BLUE'] = "\033[1;34m"
COLORS['LIGHT_PURPLE'] = "\033[1;35m"
COLORS['LIGHT_CYAN'] = "\033[1;36m"
COLORS['LIGHT_WHITE'] = "\033[1;37m"
COLORS['BOLD'] = "\033[1m"
COLORS['FAINT'] = "\033[2m"
COLORS['ITALIC'] = "\033[3m"
COLORS['UNDERLINE'] = "\033[4m"
COLORS['BLINK'] = "\033[5m"
COLORS['NEGATIVE'] = "\033[7m"
COLORS['CROSSED'] = "\033[9m"
COLORS['END'] = "\033[0m"

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
ROS_VERSION = os.getenv('ROS_VERSION')
if (ROS_VERSION is None):
    print(
        COLORS['RED'] + COLORS['BLINK'] + COLORS['BOLD'] +
        "Unknown ROS Verison! Please install the ros_environment package and source the /opt/ros/YOUR-ROS-VERSION/setup.bash"
        + COLORS['END'])
    print(ROS_VERSION)
    exit()
ROS_VERSION = int(ROS_VERSION)


def run_cmd(cmd):
    print(f"{COLORS['BOLD']}{' '.join(cmd)}{COLORS['END']}")
    subprocess.call(cmd)


def main():
    print(COLORS['GREEN'] + COLORS['BOLD'] + "Install " + PKG_NAME +
          COLORS['END'] + "\n")

    if ROS_VERSION == 1:
        run_cmd(cmd=[
            "cp",
            os.path.join(CURRENT_DIR, "package_xml/package.xml.ros1"),
            os.path.join(CURRENT_DIR, "../package.xml")
        ])
    elif ROS_VERSION == 2:
        run_cmd(cmd=[
            "cp",
            os.path.join(CURRENT_DIR, "package_xml/package.xml.ros2"),
            os.path.join(CURRENT_DIR, "../package.xml")
        ])

    print("\n" + COLORS['GREEN'] + COLORS['BOLD'] + "Install " + PKG_NAME +
          " finish\n" + COLORS['END'])


if __name__ == '__main__':
    main()
