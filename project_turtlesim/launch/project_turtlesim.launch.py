from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

def generate_launch_description():
    turtlesim_ns = LaunchConfiguration('turtlesim_ns')

    turtlesim_ns_launch_arg = DeclareLaunchArgument(
        'turtlesim_ns',
        default_value='turtlesim1'
    )

    turtlesim_node = Node(
        package='turtlesim',
        namespace=turtlesim_ns,
        executable='turtlesim_node',
        name='sim'
    )

    spawn_turtle = ExecuteProcess(
        cmd=[[
            'ros2 service call ',
            turtlesim_ns,
            '/spawn ',
            'turtlesim/srv/Spawn ',
            '"{x: 2, y: 2, theta: 0.2}"'
        ]],
        shell=True
    )

    turtle1_node = Node(
        package='project_turtlesim',
        executable='robot',
        name='node_robot_1',
        parameters=[
            {'id': 1},
            {'checkpoints': [2.0, 5.0, 8.0, 5.0]}
        ]
    )

    turtle2_node = Node(
        package='project_turtlesim',
        executable='robot',
        name='node_robot_2',
        parameters=[
            {'id': 2},
            {'checkpoints': [5.0, 8.0, 5.0, 2.0]}
        ]
    )

    return LaunchDescription([
        turtlesim_ns_launch_arg,
        turtlesim_node,
        spawn_turtle,
        turtle1_node,
        turtle2_node
    ])