"""
Launch MoveIt 2 move_group + RViz for the 5-DOF lathe arm.

Usage (sim already running with controllers active):
  ros2 launch lathe_arm_moveit moveit.launch.py

Usage (standalone visualisation, no Gazebo):
  ros2 launch lathe_arm_moveit moveit.launch.py sim:=false
"""

import os
import subprocess
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def load_yaml(package_name: str, file_path: str) -> dict:
    pkg = get_package_share_directory(package_name)
    with open(os.path.join(pkg, file_path)) as f:
        return yaml.safe_load(f)


def generate_launch_description():
    pkg_desc   = get_package_share_directory("lathe_arm_description")
    pkg_moveit = get_package_share_directory("lathe_arm_moveit")

    urdf_xacro = os.path.join(pkg_desc, "urdf", "lathe_arm.urdf.xacro")
    robot_desc = subprocess.run(
        ["xacro", urdf_xacro], capture_output=True, text=True
    ).stdout

    srdf_path = os.path.join(pkg_moveit, "config", "lathe_arm.srdf")
    with open(srdf_path) as f:
        srdf_content = f.read()

    robot_description         = {"robot_description": robot_desc}
    robot_description_semantic = {"robot_description_semantic": srdf_content}
    kinematics_yaml           = load_yaml("lathe_arm_moveit", "config/kinematics.yaml")
    joint_limits_yaml         = load_yaml("lathe_arm_moveit", "config/joint_limits.yaml")
    ompl_yaml                 = load_yaml("lathe_arm_moveit", "config/ompl_planning.yaml")
    controllers_yaml          = load_yaml("lathe_arm_moveit", "config/moveit_controllers.yaml")

    planning_pipeline = {
        "planning_pipelines": ["ompl"],
        "ompl": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            **ompl_yaml,
        },
    }

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            {"robot_description_kinematics": kinematics_yaml},
            {"robot_description_planning": joint_limits_yaml},
            planning_pipeline,
            controllers_yaml,
            {"use_sim_time": True},
        ],
    )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description, {"use_sim_time": True}],
    )

    rviz_config = os.path.join(pkg_moveit, "config", "moveit.rviz")
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
        parameters=[robot_description, robot_description_semantic, {"use_sim_time": True}],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("sim", default_value="true"),
        rsp,
        move_group,
        rviz,
    ])
