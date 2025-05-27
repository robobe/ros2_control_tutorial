import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
import xacro

PKG_DESCRIPTION = "tutorial_description"
PKG_BRINGUP = ""
ROBOT = "robot.xacro"

URDF = "urdf"
CONFIG = "config"


def generate_launch_description():
    ld = LaunchDescription()

    pkg_path = os.path.join(get_package_share_directory(PKG_DESCRIPTION))
    xacro_file = os.path.join(pkg_path, URDF, ROBOT)
    robot_description_config = xacro.process_file(xacro_file).toxml()
    params = {"robot_description": robot_description_config, "use_sim_time": True}

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[params],
    )

    node_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="screen",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d",
            PathJoinSubstitution(
                [get_package_share_directory(PKG_DESCRIPTION), CONFIG, "display.rviz"]
            ),
        ],
    )

    ld.add_action(node_robot_state_publisher)
    ld.add_action(node_state_publisher_gui)
    ld.add_action(rviz_node)
    return ld
