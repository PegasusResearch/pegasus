#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    
    # ----------------------------------------
    # ---- DECLARE THE LAUNCH ARGUMENTS ------
    # ----------------------------------------
    
    # Namespace and ID of the vehicle as parameter received by the launch file
    id_arg = DeclareLaunchArgument('vehicle_id', default_value='1', description='Drone ID in the network')
    namespace_arg = DeclareLaunchArgument('vehicle_ns', default_value='drone', description='Namespace to append to every topic and node name')

    declare_autostart_cmd = DeclareLaunchArgument('autostart', default_value='true', description='Automatically startup the paths and control stack')

    # Get the name of the .yaml configuration file either from the package or an external source
    lifecycle_nodes = ['pegasus_paths']

    # Create the actual mavlink_driver_node
    manager_node = Node(
        package='pegasus_manager',
        namespace=[
            LaunchConfiguration('vehicle_ns'), 
            LaunchConfiguration('vehicle_id')],
        executable='pegasus_manager',
        name='pegasus_manager',
        output="screen",
        emulate_tty=True,
        parameters=[
            # Pass the file which contains the topics configuration
            {'node_names': lifecycle_nodes},
            {'autostart': LaunchConfiguration('autostart')}
        ]
    )
        
    # Return the node to be launched by ROS2
    return LaunchDescription([
        # Launch arguments
        id_arg,
        namespace_arg,
        declare_autostart_cmd,
        # Launch files
        manager_node
    ])