"""
Launch the 5-DOF lathe arm in Gazebo Harmonic with ros2_control.

  ros2 launch lathe_arm_bringup sim.launch.py
"""

import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_desc    = get_package_share_directory("lathe_arm_description")
    pkg_bringup = get_package_share_directory("lathe_arm_bringup")
    pkg_gz_sim  = get_package_share_directory("ros_gz_sim")

    urdf_xacro = os.path.join(pkg_desc, "urdf", "lathe_arm.urdf.xacro")
    ctrl_yaml  = os.path.join(pkg_bringup, "config", "controllers.yaml")

    robot_desc = subprocess.run(
        ["xacro", urdf_xacro], capture_output=True, text=True
    ).stdout

    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": "-r empty.sdf"}.items(),
    )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_desc, "use_sim_time": True}],
        output="screen",
    )

    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-topic", "robot_description", "-name", "lathe_arm"],
        output="screen",
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock"],
    )

    ctrl_spawner = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
            )
        ],
    )

    arm_spawner = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["lathe_arm_controller", "--controller-manager", "/controller_manager"],
            )
        ],
    )

    return LaunchDescription([gz, rsp, spawn, bridge, ctrl_spawner, arm_spawner])
