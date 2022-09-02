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
    topics_yaml_arg = DeclareLaunchArgument(
        'topics_yaml', 
        default_value=os.path.join(get_package_share_directory('pegasus_paths'), 'config', 'topics.yaml'),
        description='The topic names assigned for the control loop')

    # Get the name of the .yaml configuration file either from the package or an external source
    gains_yaml_arg = DeclareLaunchArgument(
        'gains_yaml', 
        default_value=os.path.join(get_package_share_directory('position_pid'), 'config', 'control_gains.yaml'),
        description='The gains of the PID controller')

    # Create the actual mavlink_driver_node
    pid_node = Node(
        package='position_pid',
        namespace=[
            LaunchConfiguration('vehicle_ns'), 
            LaunchConfiguration('vehicle_id')],
        executable='position_pid',
        name='position_pid',
        output="screen",
        emulate_tty=True,
        parameters=[
            # Pass the file which contains the topics configuration
            LaunchConfiguration('topics_yaml'),
            LaunchConfiguration('gains_yaml')
        ]
    )
        
    # Return the node to be launched by ROS2
    return LaunchDescription([
        # Launch arguments
        id_arg,
        namespace_arg,
        topics_yaml_arg,
        gains_yaml_arg,
        # Launch files
        pid_node])