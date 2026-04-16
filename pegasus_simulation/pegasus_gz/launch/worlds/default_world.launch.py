#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def launch_world(context, *args, **kwargs):

    # Launch Gazebo World — pass the model path as a positional gz arg
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
            ]),
            launch_arguments={'gz_args': f"-r {get_package_share_directory('pegasus_gz')}/worlds/{LaunchConfiguration('world').perform(context)}/model.sdf"}.items(),
        )
    ]

def generate_launch_description():

    # Check if PX4_DIR is set in the environment
    if 'PX4_DIR' not in os.environ:
        raise EnvironmentError("Please set the environment variable 'PX4_DIR' to point to your PX4-Autopilot directory.")

    # Get the PX4 directory from the environment
    PX4_DIR = os.environ['PX4_DIR']

    pkg_pegasus_gz = get_package_share_directory('pegasus_gz')
    pegasus_gz_worlds = os.path.join(pkg_pegasus_gz, 'worlds')
    pegasus_gz_models = os.path.join(pkg_pegasus_gz, 'models')

    # Setup the gazebo path to include PX4 models and worlds
    px4_gz_models = os.path.join(PX4_DIR, 'Tools', 'simulation', 'gz', 'models')
    px4_gz_worlds = os.path.join(PX4_DIR, 'Tools', 'simulation', 'gz', 'worlds')
    px4_gz_plugins = os.path.join(PX4_DIR, 'build', 'px4_sitl_default', 'src', 'modules', 'simulation', 'gz_plugins')
    px4_gz_server_config = os.path.join(PX4_DIR, 'src', 'modules', 'simulation', 'gz_bridge', 'server.config')

    # Setup GZ_SIM environment variables
    gz_sim_resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    gz_sim_system_plugin_path = os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')

    return LaunchDescription([
        # Environment variables
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', f"{gz_sim_resource_path}:{px4_gz_models}:{px4_gz_worlds}:{pegasus_gz_models}:{pegasus_gz_worlds}"),
        SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', f"{gz_sim_system_plugin_path}:{px4_gz_plugins}"),
        SetEnvironmentVariable('GZ_SIM_SERVER_CONFIG_PATH', px4_gz_server_config),
        SetEnvironmentVariable('GZ_TRANSPORT_LOCALHOST_ONLY','1'),
        SetEnvironmentVariable('IGN_TRANSPORT_DISABLE_MULTICAST', '1'),
        SetEnvironmentVariable('GZ_IP', '127.0.0.1'),
        
        # Get the specified world from the pegasus_gz worlds folder 
        DeclareLaunchArgument('world', default_value='default', description='The name of the world to launch inside the pegasus_gz worlds folder.'),

        # Launch the actual gazebo with the world
        OpaqueFunction(function=launch_world)
    ])