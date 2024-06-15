from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import TextSubstitution, LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from ament_index_python import get_package_share_directory
import os
import yaml

def read_yaml(file_path: str, prefix: str):
    with open(file_path, 'r') as f:
        data = yaml.safe_load(f)
        params = data[prefix]['ros__parameters']
    return params

def launch_setup(context, *args, **kwargs):

    load_composable_nodes = LoadComposableNodes(
        target_container=LaunchConfiguration("container_name"),
        composable_node_descriptions=[
            ComposableNode(
                package="sdf_generator_ros",
                plugin="sdf_generator::TsdfServer",
                name="tsdf_server",
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
        ],
    )

    return [
        load_composable_nodes
    ]

def generate_launch_description():
    arg_container_name = DeclareLaunchArgument(
        "container_name", default_value=TextSubstitution(text="my_container")
    )

    return LaunchDescription([
        arg_container_name,
        OpaqueFunction(function=launch_setup)
    ])