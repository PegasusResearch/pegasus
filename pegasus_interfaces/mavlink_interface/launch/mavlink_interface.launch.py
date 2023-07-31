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
    
    # Define the drone MAVLINK IP and PORT
    mav_connection_arg = DeclareLaunchArgument('connection', default_value='udp://:14550', description='The interface used to connect to the vehicle')
    
    # Define the drone MAVLINK forward ips and ports
    mavlink_forward_arg = DeclareLaunchArgument('mavlink_forward', default_value="['udp://127.0.0.1:14551']", description='A list of ips where to forward mavlink messages')

    # Define which file to use for the drone parameters
    # drone_params_yaml_arg = DeclareLaunchArgument(
    #     'drone_params', 
    #     default_value=os.path.join(get_package_share_directory('mavlink_interface'), 'config', 'drone_params.yaml'),
    #     description='The directory where the drone parameters such as mass, thrust curve, etc. are defined')

    # Get the name of the .yaml configuration file either from the package or an external source
    topics_yaml_arg = DeclareLaunchArgument(
        'mavlink_topics_yaml', 
        default_value=os.path.join(get_package_share_directory('mavlink_interface'), 'config', 'topics.yaml'),
        description='The topic names assigned inside the mavlink interface')

    # Create the actual mavlink_interface_node
    mavlink_interface_node = Node(
        package='mavlink_interface',
        namespace=[
            LaunchConfiguration('vehicle_ns'), 
            LaunchConfiguration('vehicle_id')],
        executable='mavlink_interface',
        name='mavlink_interface',
        output="screen",
        emulate_tty=True,
        parameters=[
            # Pass the file which contains the topics configuration and rates for telemetry
            LaunchConfiguration('mavlink_topics_yaml'), 
            #LaunchConfiguration('drone_params'),
            # Pass the connection URL (udp, tcp or serial)
            # as well as the mavlink forward ips (for example for operating QGroundControl in parallel)
            {
                'connection': LaunchConfiguration('connection'), 
                'mavlink_forward': LaunchConfiguration('mavlink_forward')
            }
        ]
    )
        
    # Return the node to be launched by ROS2
    return LaunchDescription([
        # Launch arguments
        id_arg,
        namespace_arg,
        mav_connection_arg,
        mavlink_forward_arg,
        #drone_params_yaml_arg,
        topics_yaml_arg,
        # Launch files
        mavlink_interface_node])