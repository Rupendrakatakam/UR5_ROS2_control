import os
import socket

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def get_local_ip():
    """Get the primary local IP address."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(('8.8.8.8', 80))
        ip = s.getsockname()[0]
    except Exception:
        ip = '127.0.0.1'
    finally:
        s.close()
    return ip


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    local_ip = get_local_ip()

    # Gazebo environment — force transport to use correct interface
    gazebo_env = {
        'GZ_IP': local_ip,
        'GZ_SIM_RESOURCE_PATH': os.pathsep.join([
            os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
            '/usr/share/gz/gz-sim8/worlds',
        ]),
        'GZ_SIM_SYSTEM_PLUGIN_PATH': os.pathsep.join([
            os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', ''),
            os.environ.get('LD_LIBRARY_PATH', ''),
        ]),
    }

    # URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare('ur5_control'),
                 'urdf', 'ur5e_gazebo.urdf.xacro']
            ),
        ]
    )
    robot_description = {'robot_description': robot_description_content}

    # Controller config
    robot_controllers = PathJoinSubstitution(
        [FindPackageShare('ur5_control'), 'config', 'ur5e_controllers.yaml']
    )

    # Gazebo server (headless — RViz2 handles visualization)
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-s', 'empty.sdf'],
        output='screen',
        additional_env=gazebo_env,
    )

    # Robot state publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}]
    )

    # Spawn UR5e into Gazebo
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description',
                   '-name', 'ur5e', '-allow_renaming', 'true'],
        additional_env={'GZ_IP': local_ip},
    )

    # Spawner: joint_state_broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )

    # Spawner: joint_trajectory_controller
    joint_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_trajectory_controller',
            '--param-file',
            robot_controllers,
        ],
    )

    # Bridge: Gazebo clock -> ROS
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
        additional_env={'GZ_IP': local_ip},
    )

    # RViz2
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution(
            [FindPackageShare('ur_description'), 'rviz', 'view_robot.rviz'])],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='If true, use simulated clock'),

        # Gazebo server
        gazebo,

        # Spawn robot after Gazebo starts (with delay for transport to initialize)
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=gazebo,
                on_start=[
                    TimerAction(
                        period=5.0,
                        actions=[gz_spawn_entity],
                    )
                ],
            )
        ),

        # Load joint_state_broadcaster after robot spawns
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),

        # Load joint_trajectory_controller after JSB loads
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[joint_trajectory_controller_spawner],
            )
        ),

        # Clock bridge
        bridge,

        # Robot state publisher
        node_robot_state_publisher,

        # RViz
        rviz,
    ])
