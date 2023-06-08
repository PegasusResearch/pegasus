import os
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():

    # Create the actual mavlink_interface_node
    midair_dataset_player_node = Node(
        package='midair_dataset_player',
        namespace='drone1',
        executable='midair_dataset_player',
        name='midair_dataset_player',
        output="screen",
        emulate_tty=True,
        parameters=[])
        
    # Return the node to be launched by ROS2
    return LaunchDescription([midair_dataset_player_node])