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

        # Position and orientation of the vehicle in the Gazebo simulation (expressed in ENU)
        DeclareLaunchArgument('x', default_value='0.0', description='X position expressed in ENU'),
        DeclareLaunchArgument('y', default_value='0.0', description='Y position expressed in ENU'),
        DeclareLaunchArgument('z', default_value='0.3', description='Z position expressed in ENU'),
        DeclareLaunchArgument('R', default_value='0.0', description='Roll orientation expressed in ENU'),
        DeclareLaunchArgument('P', default_value='0.0', description='Pitch orientation expressed in ENU'),
        DeclareLaunchArgument('Y', default_value='0.0', description='Yaw orientation expressed in ENU'),

        # Launch the vehicle model in Gazebo (which also launches the corresponding PX4 SITL instance)
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('pegasus_gz'), 'launch/vehicles/pegasus_vehicle.launch.py')),
            launch_arguments={
                'vehicle_id': LaunchConfiguration('vehicle_id'),
                'vehicle_ns': LaunchConfiguration('vehicle_ns'),
                'x': LaunchConfiguration('x'),
                'y': LaunchConfiguration('y'),
                'z': LaunchConfiguration('z'),
                'R': LaunchConfiguration('R'),
                'P': LaunchConfiguration('P'),                
                'Y': LaunchConfiguration('Y')
            }.items(),
        ),
        
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