import os
import pathlib
import launch
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from launch.actions import OpaqueFunction

def launch_setup(context, *args, **kwargs):
    joint_controllers = []
    joint_controllers.append(LaunchConfiguration("joint0_controller"))
    joint_controllers.append(LaunchConfiguration("joint1_controller"))
    joint_controllers.append(LaunchConfiguration("joint2_controller"))
    joint_controllers.append(LaunchConfiguration("joint3_controller"))
    joint_controllers.append(LaunchConfiguration("joint4_controller"))
    joint_controllers.append(LaunchConfiguration("joint5_controller"))
    joint_controllers.append(LaunchConfiguration("joint6_controller"))
    joint_controllers.append(LaunchConfiguration("joint7_controller"))
    joint_controllers.append(LaunchConfiguration("joint8_controller"))
    joint_controllers.append(LaunchConfiguration("joint9_controller"))
    joint_controllers.append(LaunchConfiguration("joint10_controller"))
    joint_controllers.append(LaunchConfiguration("joint11_controller"))
    world = LaunchConfiguration('world')
    robot_urdf = LaunchConfiguration('robot_urdf').perform(context)
    ros2_control_params_file = LaunchConfiguration('ros_control_config').perform(context)

    number_of_motors =  int(LaunchConfiguration('number_of_motors').perform(context))

    urdf_dir = get_package_share_directory('webots_description')
    yaml_dir = get_package_share_directory('webots_bringup')
    resource_dir = get_package_share_directory('webots_simulation_assets')


    
    robot_description = pathlib.Path(os.path.join(urdf_dir, 'urdf', robot_urdf)).read_text()
    ros2_control_params = os.path.join(yaml_dir, 'config', 'webot_dof12_controller.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    webots = WebotsLauncher(
        world=PathJoinSubstitution([resource_dir, 'worlds', world])
    )

    controller_manager_timeout = ['--controller-manager-timeout', ' 50']
    controller_manager_prefix = 'python.exe' if os.name == 'nt' else ''

    joint_controller_spawners = []
    for i in range(number_of_motors):
        joint_controller_spawners.append(Node(
            package="controller_manager",
            executable="spawner.py",
            output='screen',
            arguments= [joint_controllers[i]] + controller_manager_timeout,
            prefix=controller_manager_prefix,
            parameters=[{'use_sim_time': use_sim_time}],
        ))

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner.py',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['joint_state_broadcaster'] + controller_manager_timeout,
        parameters=[
            {'use_sim_time': use_sim_time},
        ],
    )

    agiletaur_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen', 
        parameters=[
            {'robot_description': robot_description,
             'use_sim_time': use_sim_time},
            ros2_control_params
        ],
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': '<robot name=""><link name=""/></robot>'
        }],
    )
    nodes = joint_controller_spawners + [
             joint_state_broadcaster_spawner,
             webots,
             robot_state_publisher,
             agiletaur_driver,
             launch.actions.RegisterEventHandler(
                event_handler=launch.event_handlers.OnProcessExit(
                    target_action=webots,
                    on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
                ))
            ]

    
    return nodes

def generate_launch_description():    
    return launch.LaunchDescription([
        DeclareLaunchArgument('number_of_motors', default_value='12'),
        DeclareLaunchArgument('joint0_controller', default_value='joint0_velocity_controller'),
        DeclareLaunchArgument('joint1_controller', default_value='joint1_velocity_controller'),
        DeclareLaunchArgument('joint2_controller', default_value='joint2_velocity_controller'),
        DeclareLaunchArgument('joint3_controller', default_value='joint3_velocity_controller'),
        DeclareLaunchArgument('joint4_controller', default_value='joint4_velocity_controller'),
        DeclareLaunchArgument('joint5_controller', default_value='joint5_velocity_controller'),
        DeclareLaunchArgument('joint6_controller', default_value='joint6_velocity_controller'),
        DeclareLaunchArgument('joint7_controller', default_value='joint7_velocity_controller'),
        DeclareLaunchArgument('joint8_controller', default_value='joint8_velocity_controller'),
        DeclareLaunchArgument('joint9_controller', default_value='joint9_velocity_controller'),
        DeclareLaunchArgument('joint10_controller', default_value='joint10_velocity_controller'),
        DeclareLaunchArgument('joint11_controller', default_value='joint11_velocity_controller'),
        DeclareLaunchArgument('world', default_value='dof12.wbt'),
        DeclareLaunchArgument('robot_urdf', default_value='dof12.urdf'),
        DeclareLaunchArgument('ros_control_config', default_value='webot_dof12_controller.yaml'),
        OpaqueFunction(function = launch_setup)
        ])