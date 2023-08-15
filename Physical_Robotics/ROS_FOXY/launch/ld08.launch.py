from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        Node(
            package='ld08_driver',
            node_executable='ld08_driver',
            node_name='ld08_driver',
            node_namespace='r2tb_02',  # <------------------- ADD THIS!
            output='screen'),
    ])
