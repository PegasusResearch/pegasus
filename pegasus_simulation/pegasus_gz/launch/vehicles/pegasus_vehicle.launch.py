#!/usr/bin/env python3
"""
| File: pegasus_vehicle.launch.py
| Author: Marcelo Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
| License: Non-Commercial & Non-Military BSD4 License. Copyright (c) 2026, Marcelo Jacinto. All rights reserved.
| Description: Base launch file to spawn the pegasus vehicle model in Gazebo, along with its corresponding PX4 SITL instance.
"""
import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def create_static_transforms(vehicle_ns: str, vehicle_id: int, gazebo_namespace: str) -> list:
    """Create static sensor transforms under Gazebo frame naming."""
    gazebo_base_link_frame = f'{gazebo_namespace}/base_link'
    base_link_frame = f'{vehicle_ns}{vehicle_id}/base_link'

    # Artificial 
    artificial_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '0.0', '0.0', '0.0', '0.0', '0.0', '0.0',
            base_link_frame, f'{gazebo_base_link_frame}'
        ],
        output='screen'
    )

    # Realsense camera pose from model.sdf: x=0.15, y=0.0, z=0.0, RPY=(0,0,0)
    realsense_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '0.15', '0.0', '0.0', '0.0', '0.0', '0.0',
            base_link_frame, f'{gazebo_base_link_frame}/realsense_d435i'
        ],
        output='screen'
    )

    # Downward camera pose from model.sdf: x=0.10, y=0.0, z=0.0, RPY=(0,1.5707963268,0)
    downward_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '0.10', '0.0', '0.0', '0.0', '1.5707963268', '0.0',
            base_link_frame, f'{gazebo_base_link_frame}/downward_camera'
        ],
        output='screen'
    )

    # IMU sensor is colocated with base_link, but we access to it via mavlink/xrce interface
    # therefore it already comes with the correct frame_id in the ROS2 message, so we also create a static transform for it in Gazebo to be able to visualize it in RViz and have the correct TF tree structure
    # IMU data comes in FRD (Forward-Right-Down) convention, so we also rotate the IMU frame by 180 deg about X axis
    imu_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '0.0', '0.0', '0.0',  # Translation
            '1', '0', '0', '0',   # qx qy qz qw  = 180 deg about X
            base_link_frame, f'{base_link_frame}/imu'
        ],
        output='screen'
    )

    return [artificial_tf_node, realsense_tf_node, imu_tf_node, downward_tf_node]


def launch_vehicle(context, *args, **kwargs):

    vehicle_name = 'pegasus'
    vehicle_px4_config = '4502_pg_pegasus'
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
            arguments=[
                # Format: /gazebo_topic@ros_msg_type@gazebo_msg_type
                # 1. Camera Info
                f'/world/simulation_world/model/{gazebo_namespace}/link/base_link/sensor/realsense_d435i/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                 # 2. RGB Image
                f'/world/simulation_world/model/{gazebo_namespace}/link/base_link/sensor/realsense_d435i/image@sensor_msgs/msg/Image[gz.msgs.Image',
                # 3. Depth Image
                f'/world/simulation_world/model/{gazebo_namespace}/link/base_link/sensor/realsense_d435i/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
                # 4. Point Cloud
                f'/world/simulation_world/model/{gazebo_namespace}/link/base_link/sensor/realsense_d435i/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
                # 5. Downward facing Camera info
                f'/world/simulation_world/model/{gazebo_namespace}/link/base_link/sensor/downward_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                # 6. Downward facing RGB Image
                f'/world/simulation_world/model/{gazebo_namespace}/link/base_link/sensor/downward_camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
                
            ],
            # This is where you specify the output topics
            remappings=[
                (f'/world/simulation_world/model/{gazebo_namespace}/link/base_link/sensor/realsense_d435i/camera_info', f'/{ros2_namespace}/camera/camera_info'),
                (f'/world/simulation_world/model/{gazebo_namespace}/link/base_link/sensor/realsense_d435i/image',       f'/{ros2_namespace}/camera/image_raw'),
                (f'/world/simulation_world/model/{gazebo_namespace}/link/base_link/sensor/realsense_d435i/depth_image', f'/{ros2_namespace}/camera/depth/image_raw'),
                (f'/world/simulation_world/model/{gazebo_namespace}/link/base_link/sensor/realsense_d435i/points',      f'/{ros2_namespace}/camera/pointcloud'),
                (f'/world/simulation_world/model/{gazebo_namespace}/link/base_link/sensor/downward_camera/camera_info', f'/{ros2_namespace}/downward_camera/camera_info'),
                (f'/world/simulation_world/model/{gazebo_namespace}/link/base_link/sensor/downward_camera/image',       f'/{ros2_namespace}/downward_camera/image_raw'),
            ],
            output='screen'
        )
    
    # Create static transforms for the sensors based on the Gazebo model definition
    static_tf_nodes = create_static_transforms(LaunchConfiguration('vehicle_ns').perform(context), vehicle_id, gazebo_namespace)

    return [vehicle_launch, ros2_bridge_node, *static_tf_nodes]


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
        DeclareLaunchArgument('Y', default_value='1.57', description='Yaw orientation expressed in ENU'),
    
        # Launch the actual vehicle inside gazebo, along with the corresponding PX4 SITL instance
        OpaqueFunction(function=launch_vehicle)
    ])