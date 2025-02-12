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
    estimator_manager_yaml_arg = DeclareLaunchArgument(
        'estimator_manager_yaml', 
        default_value=os.path.join(get_package_share_directory('estimator_manager'), 'config', 'estimator_manager.yaml'),
        description='The configurations for the autopilot to run')

    # Create the actual autopilot node
    estimator_manager_node = Node(
        package='estimator_manager',
        namespace=[
            LaunchConfiguration('vehicle_ns'), 
            LaunchConfiguration('vehicle_id')],
        executable='estimator_manager',
        name='estimator_manager',
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
        estimator_manager_yaml_arg,
        # Launch files
        estimator_manager_node])