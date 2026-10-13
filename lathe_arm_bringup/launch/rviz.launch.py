"""
RViz visualisation + joint_state_publisher_gui (no Gazebo).

  ros2 launch lathe_arm_bringup rviz.launch.py
"""

import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_desc   = get_package_share_directory("lathe_arm_description")
    urdf_xacro = os.path.join(pkg_desc, "urdf", "lathe_arm.urdf.xacro")

    robot_desc = subprocess.run(
        ["xacro", urdf_xacro], capture_output=True, text=True
    ).stdout

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_desc}],
    )

    jsp = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
    )

    return LaunchDescription([rsp, jsp, rviz])
