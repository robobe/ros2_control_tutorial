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

    controller_config_file = PathJoinSubstitution(
        [get_package_share_directory(PKG_BRINGUP), CONFIG, "robot_controller.yaml"]
    )

    pkg_path = os.path.join(get_package_share_directory(PKG_DESCRIPTION))
    xacro_file = os.path.join(pkg_path, URDF, ROBOT)
    robot_description = xacro.process_file(xacro_file).toxml()
    # robot_description_config = Command(['xacro ', xacro_file])
    # robot_description = ParameterValue(
    #     Command(['xacro ', xacro_file]),
    #     value_type=str
    # )
    # Create a robot_state_publisher node

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

    # when using gazebo the controller manager load when the simulation is loaded
    # controller_manager = Node(
    #     package='controller_manager',
    #     executable='ros2_control_node',
    #     parameters=[
    #         controller_config_file
    #     ],
    #     output='screen',

    # )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    velocity_control_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["velocity_controller"],
        output="screen",
    )

    effort_control_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["effort_controller"],
        output="screen",
    )

    imu_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["imu_broadcaster"],
        output="screen",
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "forward_position_controller",
            "--param-file",
            controller_config_file,
        ],
        output="screen",
    )

    gazebo_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="parameter_bridge",
        output="screen",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
    )

    bridge_file = Path(get_package_share_directory(PKG_BRINGUP)) \
        .joinpath(CONFIG) \
        .joinpath(BRIDGE_CONFIG) \
        .as_posix()

    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f"config_file:={bridge_file}"
        ],
        parameters=[
            {'use_sim_time': True},
            {'qos_overrides./imu.publisher.reliability': 'best_effort'}
        ],
    )

    ld.add_action(node_robot_state_publisher)
    ld.add_action(gazebo)
    ld.add_action(gz_spawn_entity)
    ld.add_action(gazebo_bridge)
    # ld.add_action(ros_gz_bridge)
    # ld.add_action(controller_manager)
    ld.add_action(joint_state_broadcaster_spawner)
    # ld.add_action(robot_controller_spawner)
    # ld.add_action(velocity_control_spawner)
    ld.add_action(effort_control_spawner)
    # ld.add_action(imu_broadcaster_spawner)
    return ld
