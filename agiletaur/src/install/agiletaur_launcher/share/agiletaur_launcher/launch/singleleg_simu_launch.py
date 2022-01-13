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
import yaml
from launch.actions import OpaqueFunction
       
def load_yaml_file(yaml_file_path):
    try:
        with open(yaml_file_path, 'r') as file:
            return yaml.load(file)
    except EnvironmentError as e: # parent of IOError, OSError *and* WindowsError where available
        print(str(e))
        return None
        




def launch_setup(context, *args, **kwargs):

    ## getting config path
    config_name = LaunchConfiguration('config_path').perform(context)
    config_path = os.path.join(get_package_share_directory('agiletaur_launcher'), 'config', config_name)
    print(config_path)
    # simulation or real robot
    config = load_yaml_file(config_path)
    simu_or_real = config['general']['simu_or_real']


    webots_or_motor_params = config['motor_or_webots_config'].items()
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
        launch_arguments=webots_or_motor_params
        )]

    return launch_include + nodes


def generate_launch_description():    
    return launch.LaunchDescription([
        DeclareLaunchArgument(
            'config_path',
            default_value='single_leg_simu.yaml',
            description='Choose one of config files to create different run scenarios'
        ),
        OpaqueFunction(function = launch_setup)
        ])