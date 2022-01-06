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


def generate_launch_description():

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "joint0_controller",
            default_value="joint0_velocity_controller",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "joint1_controller",
            default_value="joint1_velocity_controller",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "joint2_controller",
            default_value="joint2_velocity_controller",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "joint3_controller",
            default_value="joint3_velocity_controller",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "joint4_controller",
            default_value="joint4_velocity_controller",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "joint5_controller",
            default_value="joint5_velocity_controller",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "joint6_controller",
            default_value="joint6_velocity_controller",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "joint7_controller",
            default_value="joint7_velocity_controller",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "joint8_controller",
            default_value="joint8_velocity_controller",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "joint9_controller",
            default_value="joint9_velocity_controller",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "joint10_controller",
            default_value="joint10_velocity_controller",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "joint11_controller",
            default_value="joint11_velocity_controller",
        )
    )


    declared_arguments.append(
        DeclareLaunchArgument(
            'world',
            default_value='dof12.wbt',
            description='Choose one of the world files from `/webots_ros2_epuck/world` directory'
        )
    )
    

    joint0_controller = LaunchConfiguration("joint0_controller")
    joint1_controller = LaunchConfiguration("joint1_controller")
    joint2_controller = LaunchConfiguration("joint2_controller")
    joint3_controller = LaunchConfiguration("joint3_controller")
    joint4_controller = LaunchConfiguration("joint4_controller")
    joint5_controller = LaunchConfiguration("joint5_controller")
    joint6_controller = LaunchConfiguration("joint6_controller")
    joint7_controller = LaunchConfiguration("joint7_controller")
    joint8_controller = LaunchConfiguration("joint8_controller")
    joint9_controller = LaunchConfiguration("joint9_controller")
    joint10_controller = LaunchConfiguration("joint10_controller")
    joint11_controller = LaunchConfiguration("joint11_controller")



    urdf_dir = get_package_share_directory('webots_description')
    yaml_dir = get_package_share_directory('webots_bringup')
    resource_dir = get_package_share_directory('webots_simulation_assets')
    world = LaunchConfiguration('world')
    robot_description = pathlib.Path(os.path.join(urdf_dir, 'urdf', 'dof12.urdf')).read_text()
    ros2_control_params = os.path.join(yaml_dir, 'config', 'webot_controllers.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    webots = WebotsLauncher(
        world=PathJoinSubstitution([resource_dir, 'worlds', world])
    )

    controller_manager_timeout = ['--controller-manager-timeout', ' 50']
    controller_manager_prefix = 'python.exe' if os.name == 'nt' else ''

    joint0_controller_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        output='screen',
        arguments= [joint0_controller] + controller_manager_timeout,
        prefix=controller_manager_prefix,
        parameters=[{'use_sim_time': use_sim_time}],
    )

    joint1_controller_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        output='screen',
        arguments= [joint1_controller] + controller_manager_timeout,
        prefix=controller_manager_prefix,
        parameters=[{'use_sim_time': use_sim_time}],
    )

    joint2_controller_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        output='screen',
        arguments= [joint2_controller] + controller_manager_timeout,
        prefix=controller_manager_prefix,
        parameters=[{'use_sim_time': use_sim_time}],
    )


    joint3_controller_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        output='screen',
        arguments= [joint3_controller] + controller_manager_timeout,
        prefix=controller_manager_prefix,
        parameters=[{'use_sim_time': use_sim_time}],
    )


    joint4_controller_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        output='screen',
        arguments= [joint4_controller] + controller_manager_timeout,
        prefix=controller_manager_prefix,
        parameters=[{'use_sim_time': use_sim_time}],
    )


    joint5_controller_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        output='screen',
        arguments= [joint5_controller] + controller_manager_timeout,
        prefix=controller_manager_prefix,
        parameters=[{'use_sim_time': use_sim_time}],
    )


    joint6_controller_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        output='screen',
        arguments= [joint6_controller] + controller_manager_timeout,
        prefix=controller_manager_prefix,
        parameters=[{'use_sim_time': use_sim_time}],
    )


    joint7_controller_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        output='screen',
        arguments= [joint7_controller] + controller_manager_timeout,
        prefix=controller_manager_prefix,
        parameters=[{'use_sim_time': use_sim_time}],
    )


    joint8_controller_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        output='screen',
        arguments= [joint8_controller] + controller_manager_timeout,
        prefix=controller_manager_prefix,
        parameters=[{'use_sim_time': use_sim_time}],
    )


    joint9_controller_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        output='screen',
        arguments= [joint9_controller] + controller_manager_timeout,
        prefix=controller_manager_prefix,
        parameters=[{'use_sim_time': use_sim_time}],
    )


    joint10_controller_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        output='screen',
        arguments= [joint10_controller] + controller_manager_timeout,
        prefix=controller_manager_prefix,
        parameters=[{'use_sim_time': use_sim_time}],
    )


    joint11_controller_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        output='screen',
        arguments= [joint11_controller] + controller_manager_timeout,
        prefix=controller_manager_prefix,
        parameters=[{'use_sim_time': use_sim_time}],
    )


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

    # epuck_process = Node(
    #     package='webots_ros2_epuck',
    #     executable='epuck_node',
    #     output='screen',
    #     parameters=[
    #         {'use_sim_time': use_sim_time},
    #     ],
    # )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': '<robot name=""><link name=""/></robot>'
        }],
    )
    nodes = [
             joint0_controller_spawner,
             joint1_controller_spawner,
             joint2_controller_spawner,
             joint3_controller_spawner,
             joint4_controller_spawner,
             joint5_controller_spawner,
             joint6_controller_spawner,
             joint7_controller_spawner,
             joint8_controller_spawner,
             joint9_controller_spawner,
             joint10_controller_spawner,
             joint11_controller_spawner,
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

    
    return LaunchDescription(declared_arguments + nodes)