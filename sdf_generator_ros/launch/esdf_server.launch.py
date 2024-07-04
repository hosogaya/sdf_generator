from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction, LogInfo
from launch_ros.actions import Node, ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch.substitutions import TextSubstitution, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import UnlessCondition
import os

def launch_setup(context, *args, **kwargs):
    container_name = "sdf_generator_container"

    container = ComposableNodeContainer(
        name=container_name, 
        package="rclcpp_components",
        executable="component_container",
        namespace="", 
        emulate_tty=True, # needed for display of logs
        output='screen'
    )
    
    tsdf_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory("sdf_generator_ros"), "/launch/load_esdf_server.launch.py"]), 
        launch_arguments={
            "container_name": TextSubstitution(text=container_name), 
            "tsdf_param_file_path": TextSubstitution(text=os.path.join(get_package_share_directory("sdf_generator_ros"), 'config/tsdf_server.param.yaml')), 
            "esdf_param_file_path": TextSubstitution(text=os.path.join(get_package_share_directory("sdf_generator_ros"), 'config/esdf_server.param.yaml')), 
            "remap_file_path": TextSubstitution(text=os.path.join(get_package_share_directory("sdf_generator_ros"), 'config', 'remapping.yaml'))
        }.items()
    )
    
    return [container, tsdf_server]
 
def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])