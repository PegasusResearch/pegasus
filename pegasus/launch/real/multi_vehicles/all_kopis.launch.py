#!/usr/bin/env python3
import os
import sys
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    
    # ----------------------------------------
    # ---- DECLARE THE LAUNCH ARGUMENTS ------
    # ----------------------------------------

    # Namespace and ID of the vehicle as parameter received by the launch file
    namespace_arg = DeclareLaunchArgument('vehicle_ns', default_value='drone', description='Namespace to append to every topic and node name')
    
    # Define the drone MAVLINK forward ips and ports
    mavlink_forward_arg = DeclareLaunchArgument('mavlink_forward', default_value="['']", description='A list of ips where to forward mavlink messages')
    
    # Define which file to use for the drone parameters
    drone_params_file_arg = DeclareLaunchArgument(
        'drone_params', 
        default_value=os.path.join(get_package_share_directory('pegasus'), 'config', 'kopis.yaml'),
        description='The directory where the drone parameters such as mass, thrust curve, etc. are defined')
    
    # ----------------------------------------
    # ---- DECLARE THE NODES TO LAUNCH -------
    # ----------------------------------------

    # Call the MOCAP driver launch file
    mocap_launch_file = IncludeLaunchDescription(
        # Grab the launch file for the mocap interface
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('mocap_interface'), 'launch/vrpn.launch.py')),
        # Define costume launch arguments/parameters used for the mocap interface
        launch_arguments={
            'id': '7', 
            'namespace': LaunchConfiguration('vehicle_ns')
        }.items(),
        #condition=LaunchConfigurationEquals('activate_mocap', 'True')
    )
    
    # Call MAVLINK interface package launch file 
    mavlink_interface_launch_file_kopis7 = IncludeLaunchDescription(
        # Grab the launch file for the mavlink interface
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('mavlink_interface'), 'launch/mavlink_interface.launch.py')),
        # Define costume launch arguments/parameters used for the mavlink interface
        launch_arguments={
            'vehicle_id': '7', 
            'vehicle_ns': LaunchConfiguration('vehicle_ns'),
            'drone_params': LaunchConfiguration('drone_params'),
            'connection': 'udp://:15007',
            'mavlink_forward': LaunchConfiguration('mavlink_forward'),
        }.items(),
    )

    # Call autopilot package launch file
    autopilot_launch_file_kopis7 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('autopilot'), 'launch/autopilot.launch.py')),
        # Define costume launch arguments/parameters used 
        launch_arguments={
            'vehicle_id': '7',
            'vehicle_ns': LaunchConfiguration('vehicle_ns'),
            'autopilot_yaml': LaunchConfiguration('drone_params'),
        }.items(),
    )

    # Call MAVLINK interface package launch file 
    mavlink_interface_launch_file_kopis8 = IncludeLaunchDescription(
        # Grab the launch file for the mavlink interface
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('mavlink_interface'), 'launch/mavlink_interface.launch.py')),
        # Define costume launch arguments/parameters used for the mavlink interface
        launch_arguments={
            'vehicle_id': '8', 
            'vehicle_ns': LaunchConfiguration('vehicle_ns'),
            'drone_params': LaunchConfiguration('drone_params'),
            'connection': 'udp://:15008',
            'mavlink_forward': LaunchConfiguration('mavlink_forward'),
        }.items(),
    )

    # Call autopilot package launch file
    autopilot_launch_file_kopis8 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('autopilot'), 'launch/autopilot.launch.py')),
        # Define costume launch arguments/parameters used 
        launch_arguments={
            'vehicle_id': '8',
            'vehicle_ns': LaunchConfiguration('vehicle_ns'),
            'autopilot_yaml': LaunchConfiguration('drone_params'),
        }.items(),
    )

    # Call MAVLINK interface package launch file 
    mavlink_interface_launch_file_kopis9 = IncludeLaunchDescription(
        # Grab the launch file for the mavlink interface
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('mavlink_interface'), 'launch/mavlink_interface.launch.py')),
        # Define costume launch arguments/parameters used for the mavlink interface
        launch_arguments={
            'vehicle_id': '9', 
            'vehicle_ns': LaunchConfiguration('vehicle_ns'),
            'drone_params': LaunchConfiguration('drone_params'),
            'connection': 'udp://:15009',
            'mavlink_forward': LaunchConfiguration('mavlink_forward'),
        }.items(),
    )

    # Call autopilot package launch file
    autopilot_launch_file_kopis9 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('autopilot'), 'launch/autopilot.launch.py')),
        # Define costume launch arguments/parameters used 
        launch_arguments={
            'vehicle_id': '9',
            'vehicle_ns': LaunchConfiguration('vehicle_ns'),
            'autopilot_yaml': LaunchConfiguration('drone_params'),
        }.items(),
    )

    # Call MAVLINK interface package launch file 
    mavlink_interface_launch_file_kopis10 = IncludeLaunchDescription(
        # Grab the launch file for the mavlink interface
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('mavlink_interface'), 'launch/mavlink_interface.launch.py')),
        # Define costume launch arguments/parameters used for the mavlink interface
        launch_arguments={
            'vehicle_id': '10', 
            'vehicle_ns': LaunchConfiguration('vehicle_ns'),
            'drone_params': LaunchConfiguration('drone_params'),
            'connection': 'udp://:15010',
            'mavlink_forward': LaunchConfiguration('mavlink_forward'),
        }.items(),
    )

    # Call autopilot package launch file
    autopilot_launch_file_kopis10 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('autopilot'), 'launch/autopilot.launch.py')),
        # Define costume launch arguments/parameters used 
        launch_arguments={
            'vehicle_id': '10',
            'vehicle_ns': LaunchConfiguration('vehicle_ns'),
            'autopilot_yaml': LaunchConfiguration('drone_params'),
        }.items(),
    )

    # ----------------------------------------
    # ---- RETURN THE LAUNCH DESCRIPTION -----
    # ----------------------------------------
    return LaunchDescription([
        # Launch arguments
        namespace_arg,
        mavlink_forward_arg,
        drone_params_file_arg,
        # Launch files for kopis 7
        mavlink_interface_launch_file_kopis7,
        autopilot_launch_file_kopis7,
        # Launch files for kopis 8
        mavlink_interface_launch_file_kopis8,
        autopilot_launch_file_kopis8,
        # Launch files for kopis 9
        mavlink_interface_launch_file_kopis9,
        autopilot_launch_file_kopis9,
        # Launch files for kopis 10
        mavlink_interface_launch_file_kopis10,
        autopilot_launch_file_kopis10,
        # Launch file for the mocap interface
        mocap_launch_file
    ])