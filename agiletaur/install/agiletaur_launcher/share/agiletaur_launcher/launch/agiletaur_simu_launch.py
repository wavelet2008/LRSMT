import os
import pathlib
import launch
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory



def generate_launch_description():


    declared_arguments = []
    # declared_arguments.append(
    #     DeclareLaunchArgument(
    #         "joint0_controller",
    #         default_value="joint0_velocity_controller",
    #     )
    # )

    simu_launch_dir = get_package_share_directory('webots_bringup')
    simu_launch_file = os.path.join(simu_launch_dir, 'launch', 'webots.launch.py')
    nodes = [
        Node(
            package="agiletaur_controller",
            executable="spawner.py",
            output='screen',
            arguments= [joint7_controller] + controller_manager_timeout,
            prefix=controller_manager_prefix,
            parameters=[{'use_sim_time': use_sim_time}],
            )

    ]
    launch_include = [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(simu_launch_file))]

    return LaunchDescription(declared_arguments + launch_include)