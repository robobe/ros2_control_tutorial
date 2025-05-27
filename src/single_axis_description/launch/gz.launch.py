import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command
import xacro 

PKG_DESCRIPTION = "tutorial_description"
ROBOT = "robot.xacro"

def generate_launch_description():
    ld = LaunchDescription()

    pkg_path = os.path.join(get_package_share_directory(PKG_DESCRIPTION))
    xacro_file = os.path.join(pkg_path,'urdf',ROBOT)
    robot_description_config = xacro.process_file(xacro_file).toxml()
    #robot_description_config = Command(['xacro ', xacro_file])
    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config, 'use_sim_time': True}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])]),
        launch_arguments=[("gz_args", " -r -v 3 empty.sdf")]
    )
    
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', 'robot', '-topic', '/robot_description'],
    )
    ld.add_action(gazebo)
    ld.add_action(gz_spawn_entity)
    ld.add_action(node_robot_state_publisher)
    return ld