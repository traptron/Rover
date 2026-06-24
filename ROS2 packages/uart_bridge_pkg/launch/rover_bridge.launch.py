import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    uart_bridge_node = Node(
        package='uart_bridge_pkg',
        executable='uart_bridge_node',
        name='uart_bridge_node',
        output='screen',
        parameters=[
            {'serial_port': '/dev/ttyS7'},
            {'baudrate': 115200}
        ]
    )

    web_dashboard_node = Node(
        package='uart_bridge_pkg',
        executable='web_dashboard_node',
        name='web_dashboard_node',
        output='screen'
    )

    return LaunchDescription([
        uart_bridge_node,
        web_dashboard_node
    ])
