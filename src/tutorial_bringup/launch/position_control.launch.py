import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.parameter_descriptions import ParameterValue
import xacro
from pathlib import Path

PKG_BRINGUP = "tutorial_bringup"
PKG_DESCRIPTION = "tutorial_description"
ROBOT = "robot.xacro"
CONFIG = "config"
URDF = "urdf"
BRIDGE_CONFIG = "gz_bridge.yaml"

def generate_launch_description():
    ld = LaunchDescription()

    pkg_path = os.path.join(get_package_share_directory(PKG_DESCRIPTION))
    xacro_file = os.path.join(pkg_path, URDF, ROBOT)
    robot_description = xacro.process_file(xacro_file).toxml()

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        get_package_share_directory("ros_gz_sim"),
                        "launch",
                        "gz_sim.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments=[("gz_args", " -r -v 3 my_world.sdf")],
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-name", "robot", "-topic", "/robot_description"],
    )


    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    control_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["position_controller"],
        output="screen",
    )

    gazebo_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="parameter_bridge",
        output="screen",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
    )


    ld.add_action(node_robot_state_publisher)
    ld.add_action(gazebo)
    ld.add_action(gz_spawn_entity)
    ld.add_action(gazebo_bridge)
    ld.add_action(control_spawner)
    ld.add_action(joint_state_broadcaster_spawner)

    return ld
