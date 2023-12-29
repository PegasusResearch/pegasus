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

    # Get the name of the .yaml configuration file either from the package or an external source
    console_yaml_arg = DeclareLaunchArgument(
        'console_yaml', 
        default_value=os.path.join(get_package_share_directory('pegasus_console'), 'config', 'console_config.yaml'),
        description='The configurations for the terminal console to run')

    # Create the actual autopilot node
    console_node = Node(
        package='pegasus_console',
        namespace=[
            LaunchConfiguration('vehicle_ns'), 
            LaunchConfiguration('vehicle_id')],
        executable='pegasus_console',
        name='pegasus_console',
        output="screen",
        emulate_tty=True,
        parameters=[
            # Pass the file which contains the topics configuration and rates for telemetry
            LaunchConfiguration('console_yaml'),
            # Pass the connection URL (udp, tcp or serial)
            # as well as the mavlink forward ips (for example for operating QGroundControl in parallel)
            {
                'vehicle_id': LaunchConfiguration('vehicle_id'),
                'vehicle_ns': LaunchConfiguration('vehicle_ns')
            }
        ]
    )
        
    # Return the node to be launched by ROS2
    return LaunchDescription([
        # Launch arguments
        id_arg,
        namespace_arg,
        console_yaml_arg,
        # Launch files
        console_node])