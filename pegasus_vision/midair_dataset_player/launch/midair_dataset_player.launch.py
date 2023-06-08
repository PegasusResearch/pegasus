import sys
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

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

    # Namespace and ID of the vehicle as parameter received by the launch file
    id_arg = DeclareLaunchArgument('vehicle_id', default_value=str(vehicle_id), description='Drone ID in the network')
    namespace_arg = DeclareLaunchArgument('vehicle_ns', default_value='drone', description='Namespace to append to every topic and node name')
    dataset_path_arg = DeclareLaunchArgument('dataset_path', default_value='/home/marcelo/pegasus/MidAir/PLE_training/spring/sensor_records.hdf5', description='Path to the dataset')

    # Create the actual mavlink_interface_node
    midair_dataset_player_node = Node(
        package='midair_dataset_player',
        namespace=[LaunchConfiguration('vehicle_ns'), LaunchConfiguration('vehicle_id')],
        executable='midair_dataset_player',
        name='midair_dataset_player',
        output="screen",
        emulate_tty=True,
        parameters=[{'dataset_path': LaunchConfiguration('dataset_path')}])
        
    # Return the node to be launched by ROS2
    return LaunchDescription([
        id_arg,
        namespace_arg,
        dataset_path_arg,
        midair_dataset_player_node])