from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description() -> LaunchDescription:
    turtle1_node = Node(
        package='project_turtlebot',
        executable='talker',
        parameters=[
            {'id': 1},
            {'checkpoints': [0.0, 0.0, 3.0, 0.0]},
            {'offset_position': [0.0, 0.0]}
        ]
    )

    turtle5_node = Node(
        package='project_turtlebot',
        executable='talker',
        parameters=[
            {'id': 5},
            {'checkpoints': [1.5, 1.5, 1.5, -1.5]},
            {'offset_position': [3.0, 0.0]}
        ]
    )

    return LaunchDescription([
        turtle1_node,
        turtle5_node
    ])