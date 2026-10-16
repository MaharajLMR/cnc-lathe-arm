"""
The jib arm in its loading cell, in Gazebo, with the web HMI.

  ros2 launch lathe_arm_bringup sim.launch.py                 # Gazebo window
  ros2 launch lathe_arm_bringup sim.launch.py headless:=true  # server only (recording)

Then open http://localhost:8000 to drive it.
"""

import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    desc = get_package_share_directory("lathe_arm_description")
    bringup = get_package_share_directory("lathe_arm_bringup")
    gz_launch = os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")

    robot = subprocess.run(
        ["xacro", os.path.join(desc, "urdf", "lathe_arm.urdf.xacro"),
         "controllers:=" + os.path.join(bringup, "config", "controllers.yaml")],
        capture_output=True, text=True, check=True).stdout
    world = os.path.join(bringup, "worlds", "lathe_cell.sdf")
    headless = LaunchConfiguration("headless")
    sim = {"use_sim_time": True}

    def teach_node(name, **params):
        return Node(package="lathe_arm_teach", executable=name, parameters=[{**sim, **params}], output="screen")

    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="false"),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(gz_launch),
                                 launch_arguments={"gz_args": f"-r {world}"}.items(),
                                 condition=UnlessCondition(headless)),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(gz_launch),
                                 launch_arguments={"gz_args": f"-s -r --headless-rendering {world}"}.items(),
                                 condition=IfCondition(headless)),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": robot, **sim}], output="screen"),
        Node(package="ros_gz_sim", executable="create",
             arguments=["-string", robot, "-name", "lathe_arm"], output="screen"),
        Node(package="ros_gz_bridge", executable="parameter_bridge",
             arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                        "/cell/image@sensor_msgs/msg/Image[gz.msgs.Image",
                        "/part/attach@std_msgs/msg/Empty]gz.msgs.Empty",
                        "/part/detach@std_msgs/msg/Empty]gz.msgs.Empty"]),
        TimerAction(period=3.0, actions=[
            Node(package="controller_manager", executable="spawner",
                 arguments=["joint_state_broadcaster", "arm_controller", "gripper_controller",
                            "linkage_controller", "--controller-manager", "/controller_manager"]),
        ]),
        teach_node("linkage_node"),
        teach_node("part_grasp"),
        teach_node("pot_teleop", source="web"),
        teach_node("hmi_server"),
    ])
