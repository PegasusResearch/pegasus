#!/usr/bin/env python3
"""
| File: racetrack_world.launch.py
| Author: Marcelo Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
| License: Non-Commercial & Non-Military BSD4 License. Copyright (c) 2026, Marcelo Jacinto. All rights reserved.
| Description: Base launch file to spawn the racetrack world model in Gazebo.
"""
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    return LaunchDescription([
    
        # Launch the world using the default world launch file (which does all the heavy lifting)
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('pegasus_gz'), 'launch/worlds/default_world.launch.py')),
            launch_arguments={
                'world': 'racetrack',
            }.items()
        )
    ])