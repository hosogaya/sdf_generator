from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import TextSubstitution, LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from ament_index_python import get_package_share_directory
import os
import yaml

def read_tsdf_yaml(file_path: str):
    with open(file_path, 'r') as f:
        data = yaml.safe_load(f)
        params = data["tsdf_server"]['ros__parameters']
    return params

def read_esdf_yaml(file_path: str):
    with open(file_path, 'r') as f:
        data = yaml.safe_load(f)
        params = data["esdf_server"]['ros__parameters']
    return params

def launch_setup(context, *args, **kwargs):
    tsdf_param = read_tsdf_yaml(LaunchConfiguration("tsdf_param_file_path").perform(context))
    esdf_param = read_esdf_yaml(LaunchConfiguration("esdf_param_file_path").perform(context))
    remap = read_esdf_yaml(LaunchConfiguration("remap_file_path").perform(context))

        
    load_composable_nodes = LoadComposableNodes(
        target_container=LaunchConfiguration("container_name"),
        composable_node_descriptions=[
            ComposableNode(
                package="sdf_generator_ros",
                plugin="sdf_generator::EsdfServer",
                name="esdf_server",
                extra_arguments=[{"use_intra_process_comms": True}],
                parameters=[tsdf_param, esdf_param],
                remappings=[
                    ("input/point_cloud", remap["input"]["point_cloud"]),
                    ("output/layer", remap["output"]["layer"]),
                    ("output/mesh", remap["output"]["mesh"]), 
                ]
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
    arg_tsdf_param_file_path = DeclareLaunchArgument(
        "tsdf_param_file_path", default_value=TextSubstitution(
            text=os.path.join(
                get_package_share_directory("sdf_generator_ros"), 
                'config', 'tsdf_server.param.yaml'
            )
        )
    )
    arg_remap_file_path = DeclareLaunchArgument(
        "remap_file_path", default_value=TextSubstitution(
            text=os.path.join(
                get_package_share_directory("sdf_generator_ros"),
                'config', 'remapping.yaml'
            )
        )
    )
    
    
    arg_esdf_param_file_path = DeclareLaunchArgument(
        "esdf_param_file_path", default_value=TextSubstitution(
            text=os.path.join(
                get_package_share_directory("sdf_generator_ros"), 
                'config', 'esdf_server.param.yaml'
            )
        )
    )
    return LaunchDescription([
        arg_container_name,
        arg_tsdf_param_file_path,
        arg_esdf_param_file_path,
        arg_remap_file_path,
        OpaqueFunction(function=launch_setup)
    ])