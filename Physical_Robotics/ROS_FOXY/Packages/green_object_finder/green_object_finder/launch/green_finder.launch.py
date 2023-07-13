from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    closest_object_node = Node(
        package='green_object_finder',
        executable='closet_objects'
    )
    turn_to_object_node = Node(
        package='green_object_finder',
        executable= 'turn_to_objects'
    )


    return LaunchDescription([
        closest_object_node,
        turn_to_object_node
    ])
