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
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher


def generate_launch_description():
    package_dir = get_package_share_directory('configs')
    robot_description = pathlib.Path(os.path.join(package_dir, 'agiletaur.urdf')).read_text()
    ros2_control_params = os.path.join(package_dir, 'resource', 'ros2_control_configuration.yml')

    # The node which interacts with a robot in the Webots simulation is located in the `webots_ros2_driver` package under name `driver`.
    # It is necessary to run such a node for each robot in the simulation.
    # Typically, we provide it the `robot_description` parameters from a URDF file and `ros2_control_params` from the `ros2_control` configuration file.
    webots_robot_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        parameters=[
            {'robot_description': robot_description},
            ros2_control_params
        ],
    )

    # Often we want to publish robot transforms, so we use the `robot_state_publisher` node for that.
    # If robot model is not specified in the URDF file then Webots can help us with the URDF exportation feature.
    # Since the exportation feature is available only once the simulation has started and the `robot_state_publisher` node requires a `robot_description` parameter before we have to specify a dummy robot.
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': '<robot name=""><link name=""/></robot>'
        }],
    )

    # Standard ROS 2 launch description
    return launch.LaunchDescription([

        # Start the Webots robot driver
        webots_robot_driver,

        # Start the robot_state_publisher
        robot_state_publisher,

        # This action will kill all nodes once the Webots simulation has exited
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])