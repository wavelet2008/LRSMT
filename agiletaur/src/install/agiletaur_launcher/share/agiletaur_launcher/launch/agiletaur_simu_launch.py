'''
@Author: Ryoma Liu -- ROBOLAND 
@Date: 2021-11-21 22:01:05 
@Last Modified by: Ryoma Liu
@Last Modified time: 2021-11-28 01:02:33
'''

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
from launch.substitutions import PathJoinSubstitution, TextSubstitution



       

        




def generate_launch_description():


    declared_arguments = []

    # set the lowlevel controller，"position_controller" or "velocity_controller"
    # or "effort_controller"

    simu_params = {
        "joint0_controller": "joint0_position_controller",
        "joint1_controller": "joint1_position_controller",
        "joint2_controller": "joint2_position_controller",
        "joint3_controller": "joint3_position_controller",
        "joint4_controller": "joint4_position_controller",
        "joint5_controller": "joint5_position_controller",
        "joint6_controller": "joint6_position_controller",
        "joint7_controller": "joint7_position_controller",
        "joint8_controller": "joint8_position_controller",
        "joint9_controller": "joint9_position_controller",
        "joint10_controller": "joint10_position_controller",
        "joint11_controller": "joint11_position_controller",
        "world": "dof12.wbt",
    }.items()


    
    controller_params = {

    }.items()

    simu_launch_dir = get_package_share_directory('webots_bringup')
    simu_launch_file = os.path.join(simu_launch_dir, 'launch', 'webots.launch.py')
    nodes = [
        Node(
            package="agiletaur_controller",
            executable="agiletaur_controller",
            output='screen',
            name="agiletaur_controller"
            )

    ]
    launch_include = [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(simu_launch_file), 
        launch_arguments=simu_params)]

    return LaunchDescription(declared_arguments + launch_include + nodes)