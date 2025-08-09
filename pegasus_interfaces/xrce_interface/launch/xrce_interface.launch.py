#!/usr/bin/env python3
import os
import sys
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, ExecuteProcess, Shutdown
from launch_ros.actions import Node

def generate_launch_description():
    
    # ----------------------------------------
    # ---- DECLARE THE LAUNCH ARGUMENTS ------
    # ----------------------------------------

    # Set the connection port from the argument (limitation from ROS2 launch files)
    connection_arguments = "udp4 -p 8888 -n drone"
    for arg in sys.argv:
        if arg.startswith('connection:='):
            connection_arguments = str(arg.split(':=')[1])
    
    environment = os.environ
    environment["PX4_UXRCE_DDS_NS"] = 'drone'

    # Namespace and ID of the vehicle as parameter received by the launch file
    id_arg = DeclareLaunchArgument('vehicle_id', default_value='1', description='Drone ID in the network')
    namespace_arg = DeclareLaunchArgument('vehicle_ns', default_value='drone', description='Namespace to append to every topic and node name')
    
    # Define the drone MAVLINK IP and PORT
    connection_arg = DeclareLaunchArgument('connection', default_value=connection_arguments, description='The interface used to connect to the vehicle')
    
    # Define which file to use for the drone parameters
    drone_params_yaml_arg = DeclareLaunchArgument(
        'drone_params', 
        default_value=os.path.join(get_package_share_directory('xrce_interface'), 'config', 'drone_params.yaml'),
        description='The directory where the drone parameters such as mass, thrust curve, etc. are defined')

    # Get the name of the .yaml configuration file either from the package or an external source
    topics_yaml_arg = DeclareLaunchArgument(
        'xrce_interface_topics_yaml', 
        default_value=os.path.join(get_package_share_directory('xrce_interface'), 'config', 'topics.yaml'),
        description='The topic names assigned to use with the xrce px4 interface')

    # MicroDDS XRCE Interface program (which is not a standard ROS2 node)
    microdds_xrce_process = ExecuteProcess(
        cmd=['ros2', 'run', 'micro_xrce_vendor', 'MicroXRCEAgent'] + connection_arguments.split(' '),
        output='screen',
        shell=False,
        on_exit=Shutdown()
    )

    # Create the actual xrce_interface_node
    xrce_interface_node = Node(
        package='xrce_interface',
        namespace=[
            LaunchConfiguration('vehicle_ns'), 
            LaunchConfiguration('vehicle_id')],
        executable='xrce_interface',
        name='xrce_interface',
        output="screen",
        emulate_tty=True,
        parameters=[
            # Pass the file which contains the topics configuration and rates for telemetry
            LaunchConfiguration('xrce_interface_topics_yaml'), 
            LaunchConfiguration('drone_params'),
            # Pass the connection URL (udp, tcp or serial)
            # as well as the mavlink forward ips (for example for operating QGroundControl in parallel)
            {
                'vehicle_id': LaunchConfiguration('vehicle_id'),
                'vehicle_ns': LaunchConfiguration('vehicle_ns'),
                'connection': LaunchConfiguration('connection')
            }
        ]
    )
        
    # Return the node to be launched by ROS2
    return LaunchDescription([
        # Launch arguments
        id_arg,
        namespace_arg,
        connection_arg,
        drone_params_yaml_arg,
        topics_yaml_arg,
        # Launch files
        microdds_xrce_process,
        xrce_interface_node])