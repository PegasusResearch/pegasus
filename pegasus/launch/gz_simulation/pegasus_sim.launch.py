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

from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

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
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('mavlink_interface'), 'launch/mavlink_interface.launch.py')),
            # Define costume launch arguments/parameters used for the mavlink interface
            launch_arguments={
                'id': LaunchConfiguration('vehicle_id'), 
                'namespace': LaunchConfiguration('vehicle_ns'),
                'drone_params': LaunchConfiguration('drone_params'),
                'connection': LaunchConfiguration('connection'),
                'mavlink_forward': LaunchConfiguration('mavlink_forward')
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
        ),

        # Hardware monitor
        ComposableNodeContainer(
            name="pegasus_hardware_monitor_container",
            namespace=[
                LaunchConfiguration('vehicle_ns'), 
                LaunchConfiguration('vehicle_id')],
            package="rclcpp_components",
            executable="component_container_mt",
            composable_node_descriptions=[
                ComposableNode(
                    package="pegasus_hardware_monitor",
                    plugin="hardware_monitor::WifiMonitor",
                    name="wifi_monitor",
                    namespace=[
                        LaunchConfiguration('vehicle_ns'), 
                        LaunchConfiguration('vehicle_id')],
                    parameters=[
                        {
                            "interface": "wlp18s0",
                            "publish_rate": 0.5,
                        }
                    ],
                ),
                ComposableNode(
                    package="pegasus_hardware_monitor",
                    plugin="hardware_monitor::SysMonitor",
                    name="sys_monitor",
                    namespace=[
                        LaunchConfiguration('vehicle_ns'), 
                        LaunchConfiguration('vehicle_id')],
                    parameters=[
                        {
                            "publish_rate": 0.5,
                        }
                    ],
                ),
                ComposableNode(
                    package="pegasus_hardware_monitor",
                    plugin="hardware_monitor::BagNode",
                    name="bag_node",
                    namespace=[
                        LaunchConfiguration('vehicle_ns'), 
                        LaunchConfiguration('vehicle_id')],
                    parameters=[
                        {
                            "bag_directory": os.getenv("HOME") + "/bags",
                        }
                    ],
                ),
            ],
            output="screen",
        ),

        # Launch Rosbridge to make the ROS topics available in via websocket to the GCS (ground control station)
        Node(
            package="rosbridge_server",
            executable="rosbridge_websocket",
            name="rosbridge_websocket",
            output="screen",
            parameters=[
                {
                    # Network
                    "port": 9090,
                    "address": "0.0.0.0",
    
                    # Authentication (set to True and supply credentials if needed)
                    "authenticate": False,
    
                    # Performance
                    "max_message_size": 10_000_000,   # 10 MB – raise for images
                    "send_action_goals_in_new_thread": True,
    
                    # Fragmentation (useful for large messages over slow links)
                    "fragment_timeout": 600,
                    "delay_between_messages": 0.0,
                    "max_burst_size": 0,              # 0 = unlimited burst
    
                    # Logging verbosity: 0 = all, 1 = some, 2 = none
                    "topics_glob": "[*]",             # allow every topic
                    "services_glob": "[*]",
                    "params_glob": "[*]",
                }
            ],
        ),

        Node(
            package="rosapi",
            executable="rosapi_node",
            name="rosapi",
            output="screen",
        ),

        # Launch the web video server to stream the camera feed to the GCS (ground control station)
        Node(
            package="web_video_server",
            executable="web_video_server",
            name="web_video_server",
            parameters=[{"port": 8080, "address": "0.0.0.0"}],
        )
    ])