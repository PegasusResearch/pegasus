#!/usr/bin/env python3
"""
| File: x500_vehicle.launch.py
| Author: Marcelo Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
| License: Non-Commercial & Non-Military BSD4 License. Copyright (c) 2026, Marcelo Jacinto. All rights reserved.
| Description: Base launch file to spawn the x500 vehicle model in Gazebo, along with its corresponding PX4 SITL instance.
"""
import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def launch_vehicle(context, *args, **kwargs):

    vehicle_name = 'x500'
    vehicle_px4_config = '4500_pg_x500'
    vehicle_id = LaunchConfiguration('vehicle_id').perform(context)

    # Launch the vehicle using the default vehicle launch file (which does all the heavy lifting)
    return [
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('pegasus_gz'), 'launch/vehicles/default_vehicle.launch.py')),
            launch_arguments={
                'vehicle': vehicle_name,
                'px4_config_file': vehicle_px4_config,
                'vehicle_id': vehicle_id,
                'vehicle_ns': LaunchConfiguration('vehicle_ns').perform(context),
                'x': LaunchConfiguration('x').perform(context),
                'y': LaunchConfiguration('y').perform(context),
                'z': LaunchConfiguration('z').perform(context),
                'R': LaunchConfiguration('R').perform(context),
                'P': LaunchConfiguration('P').perform(context),
                'Y': LaunchConfiguration('Y').perform(context)
            }.items(),
        )
    ]


def generate_launch_description():

    return LaunchDescription([
        
        # Launch arguments
        DeclareLaunchArgument('vehicle_id', default_value='1', description='Drone ID in the network'),
        DeclareLaunchArgument('vehicle_ns', default_value='drone', description='Namespace to append to every topic and node name in ROS2'),
        DeclareLaunchArgument('x', default_value='0.0', description='X position expressed in ENU'),
        DeclareLaunchArgument('y', default_value='0.0', description='Y position expressed in ENU'),
        DeclareLaunchArgument('z', default_value='0.3', description='Z position expressed in ENU'),
        DeclareLaunchArgument('R', default_value='0.0', description='Roll orientation expressed in ENU'),
        DeclareLaunchArgument('P', default_value='0.0', description='Pitch orientation expressed in ENU'),
        DeclareLaunchArgument('Y', default_value='0.0', description='Yaw orientation expressed in ENU'),
    
        # Launch the actual vehicle inside gazebo, along with the corresponding PX4 SITL instance
        OpaqueFunction(function=launch_vehicle)
    ])