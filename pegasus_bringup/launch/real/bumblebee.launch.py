#!/usr/bin/env python3
import os
import sys
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    
    # --------------------------------
    # Define the vehicle ID
    # --------------------------------
    
    # Set the default vehicle id (note: this is a trick due to the parameter reading limitation in ROS2)
    default_vehicle_id = 1
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
    mav_connection_arg = DeclareLaunchArgument('connection', default_value='serial:///dev/serial/by-id/usb-Holybro_PX4_KakuteH7_0-if00:57600', description='The interface used to connect to the vehicle')

    # Define the drone MAVLINK forward ips and ports
    mavlink_forward_arg = DeclareLaunchArgument('mavlink_forward', default_value="['udp://192.168.55.100:14550']", description='A list of ips where to forward mavlink messages')
    
    # Define which file to use for the drone parameters
    drone_params_file_arg = DeclareLaunchArgument(
        'drone_params', 
        default_value=os.path.join(get_package_share_directory('pegasus_bringup'), 'config', 'iris.yaml'),
        description='The directory where the drone parameters such as mass, thrust curve, etc. are defined')
    
    # ----------------------------------------
    # ---- DECLARE THE NODES TO LAUNCH -------
    # ----------------------------------------
    
    # Call MAVLINK interface package launch file 
    mavlink_driver_launch_file = IncludeLaunchDescription(
        # Grab the launch file for the mavlink interface
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('mavlink_driver'), 'launch/mavlink_driver.launch.py')),
        # Define costume launch arguments/parameters used for the mavlink interface
        launch_arguments={
            'id': LaunchConfiguration('vehicle_id'), 
            'namespace': LaunchConfiguration('vehicle_ns'),
            'drone_params': LaunchConfiguration('drone_params'),
            'connection': LaunchConfiguration('connection'),
            'mavlink_forward': LaunchConfiguration('mavlink_forward')
        }.items(),
    )

    # Call the ueye camera interface package launch file
    ueye_camera_launch_file = IncludeLaunchDescription(
        # Grab the launch file for the ueye camera interface
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ueye_driver'), 'launch/ueye_driver.launch.py')),
        # Define costume launch arguments/parameters used for the ueye camera interface
        launch_arguments={
            'id': LaunchConfiguration('vehicle_id'),
            'namespace': LaunchConfiguration('vehicle_ns')
        }.items(),
    )

    # Call the intel real sense camera interface 
    real_sense_launch_file=IncludeLaunchDescription(
        # Grab the launch file for the ueye camera interface
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('pegasus_bringup'), 'launch/dev/realsense.launch.py')),
        # Define costume launch arguments/parameters used for the camera interface
        launch_arguments={
            'id': LaunchConfiguration('vehicle_id'), 
            'namespace': LaunchConfiguration('vehicle_ns')
        }.items(),
    )

    # Call Feature tracker interface package launch file 
    visual_odometry_launch_file = IncludeLaunchDescription(
        # Grab the launch file for the mavlink interface
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('visual_odometry'), 'launch/visual_odometry.launch.py')),
        # Define costume launch arguments/parameters used for the mavlink interface
        launch_arguments={
            'id': LaunchConfiguration('vehicle_id'), 
            'namespace': LaunchConfiguration('vehicle_ns')
        }.items(),
    )

    # ----------------------------------------
    # ---- RETURN THE LAUNCH DESCRIPTION -----
    # ----------------------------------------
    return LaunchDescription([
        # Launch arguments
        id_arg, 
        namespace_arg, 
        mav_connection_arg,
        mavlink_forward_arg,
        drone_params_file_arg,
        # Launch files
        #mavlink_driver_launch_file,
        #ueye_camera_launch_file,
        real_sense_launch_file,
        #visual_odometry_launch_file
    ])