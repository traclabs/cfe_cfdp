import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('cfdp_wrapper'),
        'config',
        'cfdp_wrapper.yaml'
        )

    node = Node(
        package='cfdp_wrapper',
        name='cfdp_wrapper',
        executable='cfdp_wrapper.py',
        parameters=[]
    )
    ld.add_action(node)
    return ld
