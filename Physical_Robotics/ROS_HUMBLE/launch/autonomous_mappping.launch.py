import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource('~/turtlebot_ws/src/r2tb01_launcher/r2tb01_bringup.launch.py'),
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource('~/turtlebot_ws/src/r2tb01_launcher/r2tb01_cartographer/cartographer.launch.py'),
        ),
    ])
