from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='auto_localization',
            executable='auto_localization_node',
            name='auto_localization',
            parameters=[
                'auto_localization/config/auto_localization.yaml'
            ]
        )
    ])
