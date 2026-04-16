#!/usr/bin/env python3
"""
| File: iris_vehicle.launch.py
| Author: Marcelo Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
| License: Non-Commercial & Non-Military BSD4 License. Copyright (c) 2026, Marcelo Jacinto. All rights reserved.
| Description: Base launch file to spawn the iris vehicle model in Gazebo, along with its corresponding PX4 SITL instance.
"""
import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def launch_vehicle(context, *args, **kwargs):

    vehicle_name = 'iris'
    vehicle_px4_config = '4501_pg_iris'
    vehicle_id = LaunchConfiguration('vehicle_id').perform(context)

    # The namespace under which the sensor topics appear in gazebo
    gazebo_namespace = f'{vehicle_name}_{vehicle_id}'

    # The namespace under which the ROS2 topics will be available
    ros2_namespace = f'{LaunchConfiguration("vehicle_ns").perform(context)}{vehicle_id}'

    # Launch the vehicle using the default vehicle launch file (which does all the heavy lifting)
    vehicle_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('pegasus_gz'), 'launch/vehicles/default_vehicle.launch.py')),
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
            }.items())
    
    # Launch the bridge and get the realsense topics bridged from gazebo to ROS2
    ros2_bridge_node = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            namespace=[
                LaunchConfiguration('vehicle_ns'), 
                LaunchConfiguration('vehicle_id')],
            arguments=[
                # Format: /gazebo_topic@ros_msg_type@gazebo_msg_type
                # 1. Camera Info
                f'/world/simulation_world/model/{gazebo_namespace}/link/base_link/sensor/front_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                 # 2. RGB Image
                f'/world/simulation_world/model/{gazebo_namespace}/link/base_link/sensor/front_camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            ],
            # This is where you specify the output topics
            remappings=[
                (f'/world/simulation_world/model/{gazebo_namespace}/link/base_link/sensor/front_camera/camera_info', f'/{ros2_namespace}/camera/camera_info'),
                (f'/world/simulation_world/model/{gazebo_namespace}/link/base_link/sensor/front_camera/image',       f'/{ros2_namespace}/camera/image_raw'),
            ],
            output='screen'
        )

    return [vehicle_launch, ros2_bridge_node]

def generate_launch_description():

    return LaunchDescription([
        
        # Launch arguments
        DeclareLaunchArgument('vehicle_id', default_value='1', description='Drone ID in the network'),
        DeclareLaunchArgument('vehicle_ns', default_value='drone', description='Namespace to append to every topic and node name in ROS2'),
        DeclareLaunchArgument('x', default_value='0.0', description='X position expressed in ENU'),
        DeclareLaunchArgument('y', default_value='0.0', description='Y position expressed in ENU'),
        DeclareLaunchArgument('z', default_value='0.2', description='Z position expressed in ENU'),
        DeclareLaunchArgument('R', default_value='0.0', description='Roll orientation expressed in ENU'),
        DeclareLaunchArgument('P', default_value='0.0', description='Pitch orientation expressed in ENU'),
        DeclareLaunchArgument('Y', default_value='0.0', description='Yaw orientation expressed in ENU'),

        # Launch the actual vehicle inside gazebo, along with the corresponding PX4 SITL instance
        OpaqueFunction(function=launch_vehicle)
    ])