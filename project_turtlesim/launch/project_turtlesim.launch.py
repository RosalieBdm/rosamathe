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

    # ! The teleportation is working, however, since the turtlesim node hasn't fully started, the teleportation
    # ! occures before the spawning of the first turtle.
    # ! In the same way, the moving of the first turtle then occures before the spawning of the second turtle.
    teleport_turtle = ExecuteProcess(
        cmd=[[
            'ros2 service call ',
            turtlesim_ns,
            '/teleport_absolute ',
            'turtlesim/srv/TeleportAbsolute ',
            '"{x: 2, y: 5, theta: 0}"'
        ]],
        shell=True
    )

    spawn_turtle = ExecuteProcess(
        cmd=[[
            'ros2 service call ',
            turtlesim_ns,
            '/spawn ',
            'turtlesim/srv/Spawn ',
            '"{x: 8, y: 5, theta: 3.14}"'
        ]],
        shell=True
    )

    turtle1_node = Node(
        package='project_turtlesim',
        executable='robot',
        name='node_robot_1',
        parameters=[
            {'id': 1},
            {'checkpoints': [8.0, 5.0, 2.0, 5.0]}
        ]
    )

    turtle2_node = Node(
        package='project_turtlesim',
        executable='robot',
        name='node_robot_2',
        parameters=[
            {'id': 2},
            {'checkpoints': [2.0, 5.0, 2.0, 8.0]}
        ]
    )

    return LaunchDescription([
        turtlesim_ns_launch_arg,
        turtlesim_node,
        teleport_turtle,
        spawn_turtle,
        turtle1_node,
        turtle2_node
    ])