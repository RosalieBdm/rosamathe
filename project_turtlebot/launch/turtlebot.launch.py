from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description() -> LaunchDescription:
    turtle1_node = Node(
        package='project_turtlebot',
        executable='talker',
        parameters=[
            {'id': 1},
            {'checkpoints': []}
        ]
    )

    turtle2_node = Node(
        package='project_turtlebot',
        executable='talker',
        parameters=[
            {'id': 5},
            {'checkpoints': []}
        ]
    )

    return LaunchDescription([
        turtle1_node,
        turtle2_node
    ])