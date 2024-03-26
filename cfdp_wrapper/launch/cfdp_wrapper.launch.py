import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


arguments = [
    DeclareLaunchArgument('namespace', default_value='/groundsystem',
                          choices=['/groundsystem', '/flightsystem'],
                          description='Where this cfdp node is running'),
    DeclareLaunchArgument('entityID', default_value='1',
                          choices=['1', '2'], description='entity ID'),
    DeclareLaunchArgument('filestore', default_value='cfdp/rosgsw',
                          description='Folder location of files transferred'),
]

def generate_launch_description():
    ld = LaunchDescription(arguments)

    config = os.path.join(
        get_package_share_directory('cfdp_wrapper'),
        'config',
        'cfdp_wrapper.yaml'
        )

    node = Node(
        package='cfdp_wrapper',
        executable='cfdp_wrapper.py',
        name='cfdp',
        namespace=LaunchConfiguration('namespace'),
        parameters=[
          {"entityID": LaunchConfiguration("entityID")},
          {"filestore": LaunchConfiguration("filestore")},          
          config
        ]
    )
    ld.add_action(node)
    return ld
