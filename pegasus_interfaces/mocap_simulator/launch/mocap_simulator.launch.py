import os
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():

    # Create the actual mavlink_interface_node
    mocap_simulator_node = Node(
        package='mocap_simulator',
        namespace='drone1',
        executable='mocap_simulator',
        name='mocap_simulator',
        output="screen",
        emulate_tty=True,
        parameters=[])
        
    # Return the node to be launched by ROS2
    return LaunchDescription([mocap_simulator_node])