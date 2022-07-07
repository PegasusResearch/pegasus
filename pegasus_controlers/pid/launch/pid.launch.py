from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # Create the actual mavlink_interface_node
    pid = Node(
        package='pid',
        namespace='drone1',
        executable='pid',
        name='pid',
        output="screen",
        emulate_tty=True)
        
    # Return the node to be launched by ROS2
    return LaunchDescription([pid])