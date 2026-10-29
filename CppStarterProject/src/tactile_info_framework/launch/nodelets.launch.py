import os
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from ament_index_python import get_package_share_directory

launch_args = [
    DeclareLaunchArgument('config', default_value='default', description='Configuration name'),
]


def launch_setup(context):
        # Declare arguments
    config_arg = LaunchConfiguration('config').perform(context)

    # Paths to configuration files
    default_config_path = os.path.join(
        get_package_share_directory('tactile_info_framework'), 'config', 'default.yaml')
    harc_config_path = os.path.join(get_package_share_directory('tactile_info_framework'), 'config', 'harc_'+ config_arg + '.yaml')

    container = ComposableNodeContainer(
            name='tactile_info_framework_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container_mt',
            composable_node_descriptions=[
                ComposableNode(
                    package='tactile_info_framework',
                    plugin='tactile_info_framework::ProcessingNodelet',
                    name='ProcessingNodelet',
                    parameters=[default_config_path, {'harc_config': harc_config_path}],
                ),
                ComposableNode(
                    package='tactile_info_framework',
                    plugin='tactile_info_framework::AcquisitionNodelet',
                    name='AcquisitionNodelet',
                    parameters=[{'harc_config': harc_config_path}],
                ),
            ],
            output='screen',
    )

    return [container]

def generate_launch_description():
    opfunc = OpaqueFunction(function = launch_setup)
    ld = LaunchDescription(launch_args)
    ld.add_action(opfunc)
    return ld
