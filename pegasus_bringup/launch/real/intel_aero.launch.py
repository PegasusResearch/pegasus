#!/usr/bin/env python3
import os
import sys
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.conditions import LaunchConfigurationEquals
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    
    # --------------------------------
    # Define the vehicle ID
    # --------------------------------
    
    # Set the default vehicle id (note: this is a trick due to the parameter reading limitation in ROS2)
    default_vehicle_id = 2
    vehicle_id = default_vehicle_id
    for arg in sys.argv:
        if arg.startswith('vehicle_id:='):
            vehicle_id = int(arg.split(':=')[1])
    
    # ----------------------------------------
    # ---- DECLARE THE LAUNCH ARGUMENTS ------
    # ----------------------------------------

    # Namespace and ID of the vehicle as parameter received by the launch file
    id_arg = DeclareLaunchArgument('vehicle_id', default_value=str(vehicle_id), description='Drone ID in the network')
    namespace_arg = DeclareLaunchArgument('vehicle_ns', default_value='drone', description='Namespace to append to every topic and node name')
    
    # Define the drone MAVLINK IP and PORT
    mav_connection_arg = DeclareLaunchArgument('connection', default_value='udp://:' + str(15000 + vehicle_id), description='The interface used to connect to the vehicle')
    # Other possible connection values
    #connection: "udp://:15006"    # snapdragon 6
    #connection: "udp://:15000"
    #connection: "serial:///dev/ttyACM0:57600"
    
    # Define which file to use for the drone parameters
    drone_params_file_arg = DeclareLaunchArgument(
        'drone_params', 
        default_value=os.path.join(get_package_share_directory('pegasus_bringup'), 'config', 'intel_aero.yaml'),
        description='The directory where the drone parameters such as mass, thrust curve, etc. are defined')
    
    # Define whether to launch the MOCAP interface or not
    mocap_connect_arg = DeclareLaunchArgument('activate_mocap', default_value='True', description='Boolean that defines whether the mocap driver is launched or not')

    # ----------------------------------------
    # ---- DECLARE THE NODES TO LAUNCH -------
    # ----------------------------------------
    
    # Call MAVLINK interface package launch file 
    mavlink_interface_launch_file = IncludeLaunchDescription(
        # Grab the launch file for the mavlink interface
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('mavlink_interface'), 'launch/mavlink_interface.launch.py')),
        # Define costume launch arguments/parameters used for the mavlink interface
        launch_arguments={
            'id': LaunchConfiguration('vehicle_id'), 
            'namespace': LaunchConfiguration('vehicle_ns'),
            'drone_params': LaunchConfiguration('drone_params'),
            'connection': LaunchConfiguration('connection')
        }.items()
    )
    
    # Call the MOCAP driver launch file
    mocap_launch_file = IncludeLaunchDescription(
        # Grab the launch file for the mocap interface
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('vrpn_client_ros'), 'launch/vrpn.launch.py')),
        # Define costume launch arguments/parameters used for the mocap interface
        launch_arguments={
            'id': LaunchConfiguration('vehicle_id'), 
            'namespace': LaunchConfiguration('vehicle_ns')
        }.items(),
        condition=LaunchConfigurationEquals('activate_mocap', 'True')
    )

    # ----------------------------------------
    # ---- RETURN THE LAUNCH DESCRIPTION -----
    # ----------------------------------------
    return LaunchDescription([
        # Launch arguments
        id_arg, 
        namespace_arg, 
        mav_connection_arg,
        drone_params_file_arg,
        mocap_connect_arg,
        # Launch files
        mavlink_interface_launch_file,
        mocap_launch_file
    ])