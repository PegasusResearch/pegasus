#!/usr/bin/env python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

    
def generate_launch_description():

    # Namespace and ID of the vehicle as parameter received by the launch file
    id_arg = DeclareLaunchArgument('vehicle_id', default_value='1', description='Drone ID in the network')
    namespace_arg = DeclareLaunchArgument('vehicle_ns', default_value='drone', description='Namespace to append to every topic and node name')

    # Define the parameters to load the iris drone from 
    realsense_params_file_arg = DeclareLaunchArgument(
        'realsense_params_yaml',
        default_value=os.path.join(get_package_share_directory('pegasus'), 'config', 'realsense_d435i.yaml'),
        description='The file where the realsense parameters are defined')

    # Use intraprocess communications (zero-copy between node images)
    intra_process_comms = DeclareLaunchArgument(
        'intra_process_comms',
        default_value='True',
        description='Zero-copy image between ROS 2 processes'
    )

    # Create the realsense node
    realsense_driver_node = Node(
        package='realsense2_camera',
        namespace=[
            LaunchConfiguration('vehicle_ns'), 
            LaunchConfiguration('vehicle_id')],
        executable='realsense2_camera_node',
        prefix=['stdbuf -o L'],
        name='camera_driver',
        output="screen",
        arguments=['--ros-args', '--log-level', 'info'],
        emulate_tty=True,
        parameters=[
            LaunchConfiguration('realsense_params_yaml'),
            LaunchConfiguration('intra_process_comms')
        ]
    )
    
    return LaunchDescription([
        # Launch arguments
        id_arg,
        namespace_arg,
        realsense_params_file_arg,
        intra_process_comms,
        # Nodes
        realsense_driver_node
    ])