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
    autopilot_yaml_arg = DeclareLaunchArgument(
        'autopilot_yaml', 
        default_value=os.path.join(get_package_share_directory('autopilot'), 'config', 'autopilot.yaml'),
        description='The configurations for the autopilot to run')

    # Create the actual autopilot node
    autopilot_node = Node(
        package='autopilot',
        namespace=[
            LaunchConfiguration('vehicle_ns'), 
            LaunchConfiguration('vehicle_id')],
        executable='autopilot',
        name='autopilot',
        output="screen",
        emulate_tty=True,
        parameters=[
            # Pass the file which contains the topics configuration and rates for telemetry
            LaunchConfiguration('autopilot_yaml'),
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
        autopilot_yaml_arg,
        # Launch files
        autopilot_node])