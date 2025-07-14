from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='task_5',
            executable='red_ball_tracker',
            name='red_ball_tracker',
            output='screen'
        )
    ])
