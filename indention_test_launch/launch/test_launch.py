import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="indention_test",
            namespace="indention_test",
            executable="tester",
            name="tester",
            remappings=[('/indention_test/ttac3', '/ttac3')]
        ),
        launch.actions.ExecuteProcess(
            cmd=[
                'ros2', 'run', 'ros2serial', 'main',
                '-p', '/dev/ttyACM0',
                '-b', '115200',
                '-n', 'bend_sensor'
            ]
        ),
        launch.actions.ExecuteProcess(
            cmd=[
                'ros2', 'run', 'ros2serial', 'main',
                '-p', '/dev/ttyACM1',
                '-b', '115200',
                '-n', 'terminal'
            ]
        ),
        launch.actions.ExecuteProcess(
            cmd=[
                'ros2', 'run', 'mf_driver', 'bend_sensor_driver'
            ]
        ),
        launch.actions.ExecuteProcess(
            cmd=[
                'ros2', 'run', 'mf_driver', 'terminal_driver'
            ]
        )
    ])