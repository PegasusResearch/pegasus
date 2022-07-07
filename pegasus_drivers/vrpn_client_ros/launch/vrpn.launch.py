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
    # Define which file to use for the drone parameters
    vrpn_params_file_arg = DeclareLaunchArgument(
        'vrpn_yaml', 
        default_value=os.path.join(get_package_share_directory('vrpn_client_ros'), 'config', 'tagus.yaml'),
        description='The configurations file for launching the VRPN node that receives data from mocap system and publishes it to ROS2')

    # Namespace and ID of the vehicle as parameter received by the launch file
    id_arg = DeclareLaunchArgument('vehicle_id', default_value='1', description='Drone ID in the network')
    namespace_arg = DeclareLaunchArgument('vehicle_ns', default_value='drone', description='Namespace to append to every topic and node name')

    # Create the actual mavlink_interface_node
    vrpn_node = Node(
        package='vrpn_client_ros',
        executable='vrpn_client_node',
        namespace=[
            LaunchConfiguration('vehicle_ns'), 
            LaunchConfiguration('vehicle_id')],
        name='vrpn_client_node',
        output="screen",
        emulate_tty=True,
        parameters=[
            # Pass the file which contains the configurations for the VRPN node to connect to the system
            LaunchConfiguration('vrpn_yaml')
        ]
    )
    
    # Return the node to be launched by ROS2
    return LaunchDescription([
        # Launch arguments
        vrpn_params_file_arg,
        # Launch files
        vrpn_node
    ])
