import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from ros_gz_bridge.actions import RosGzBridge

PACKAGE_NAME = 'tutorial_bringup'

def generate_launch_description():
    ld = LaunchDescription()

    bridge_params = os.path.join(get_package_share_directory(PACKAGE_NAME),'config','gz_bridge.yaml')
    bridge = RosGzBridge(
            bridge_name="bridge",
            config_file=bridge_params,
        )

    ld.add_action(bridge)


    return ld