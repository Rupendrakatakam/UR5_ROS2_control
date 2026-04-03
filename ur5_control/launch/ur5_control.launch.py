from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    robot_description = Command([
        'xacro ',
        PathJoinSubstitution([FindPackageShare('ur_description'), 'urdf', 'ur.urdf.xacro']),
        ' name:=ur',
        ' ur_type:=ur5e'
    ])

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[{'robot_description': robot_description}]
    )

    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('ur_description'), 'rviz', 'view_robot.rviz'
    ])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file]
    )

    # Task 1- motion control
    motion_task1_node = Node(
        package='ur5_control',
        executable='motion_task1',
        output='screen',
        prefix="xterm -e",
        parameters=[{'robot_description': robot_description}]
    )

    #task 2- keyboard control
    keyboard_task2_node = Node(
        package='ur5_control',
        executable='keyboard_task2',
        output='screen',
        prefix="xterm -e",
        parameters=[{'robot_description': robot_description}]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        rviz_node,
        motion_task1_node
        # keyboard_task2_node
    ])