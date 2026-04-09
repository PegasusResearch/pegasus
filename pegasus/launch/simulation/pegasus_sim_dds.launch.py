#!/usr/bin/env python3
"""
| File: pegasus_sim.launch.py
| Author: Marcelo Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
| License: Non-Commercial & Non-Military BSD4 License. Copyright (c) 2026, Marcelo Jacinto. All rights reserved.
| Description: Base launch file to launch the Pegasus GNC stack attached to the pegasus simulation model.
"""

import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    return LaunchDescription([
        
        # Setup the drone namespace and ID
        DeclareLaunchArgument('vehicle_id', default_value='1', description='Drone ID in the network'),
        DeclareLaunchArgument('vehicle_ns', default_value='drone', description='Namespace to append to every topic and node name'),
        
        # Define the drone MAVLINK IP and PORT
        DeclareLaunchArgument('connection', default_value='udp://:14540', description='The interface used to connect to the vehicle'),
        DeclareLaunchArgument('mavlink_forward', default_value="['']", description='A list of ips where to forward mavlink messages'),
        DeclareLaunchArgument('drone_params', default_value=os.path.join(get_package_share_directory('pegasus'), 'config/simulation', 'pegasus.yaml'), description='The directory where the drone parameters such as mass, thrust curve, etc. are defined'),
        
        # Launch the mavlink interface which is used to communicate with PX4
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('xrce_interface'), 'launch/xrce_interface.launch.py')),
            # Define costume launch arguments/parameters used for the mavlink interface
            launch_arguments={
                'id': LaunchConfiguration('vehicle_id'), 
                'namespace': LaunchConfiguration('vehicle_ns'),
                'drone_params': LaunchConfiguration('drone_params'),
                'connection': LaunchConfiguration('connection')
            }.items(),
        ),

        # Call autopilot package launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('autopilot'), 'launch/autopilot.launch.py')),
            # Define costume launch arguments/parameters used 
            launch_arguments={
                'id': LaunchConfiguration('vehicle_id'),
                'namespace': LaunchConfiguration('vehicle_ns'),
                'autopilot_yaml': LaunchConfiguration('drone_params'),
            }.items(),
        )
    ])