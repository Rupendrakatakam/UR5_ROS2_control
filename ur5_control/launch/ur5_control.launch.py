from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 1. Get the path to your URDF/Xacro file
    # urdf_path = PathJoinSubstitution([
    #     FindPackageShare('ur5_control'), 'urdf', 'ur5e_gazebo.urdf.xacro'
    # ])

    # # Command to process the xacro into a pure URDF string
    # robot_description_content = Command(
    #     [PathJoinSubstitution([FindExecutable(name='xacro')]), ' ', urdf_path]
    # )

    robot_description = Command([
        'xacro ',
        PathJoinSubstitution([FindPackageShare('ur_description'), 'urdf', 'ur.urdf.xacro']),
        ' name:=ur',
        ' ur_type:=ur5e'
    ])

    # 2. Robot State Publisher Node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[{'robot_description': robot_description}]
    )

    # 3. RViz Node (Assuming you have a saved rviz config)
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

    # 4. Our Task 1 Motion Node
    # We pass the same robot_description so our C++ KDL parser can read it
    motion_task1_node = Node(
        package='ur5_control',
        executable='motion_task1',
        output='screen',
        prefix="xterm -e", # Opens a separate terminal window so you can easily type 0 or 1!
        parameters=[{'robot_description': robot_description}]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        rviz_node,
        motion_task1_node
    ])