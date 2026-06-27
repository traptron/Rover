"""Launch file for teleoperation system (joy_node + teleop_DS4 node)."""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description for teleoperation."""

    # Get package paths
    teleop_pkg_dir = get_package_share_directory('teleop_DS4')
    config_dir = os.path.join(teleop_pkg_dir, 'config')

    # Declare launch arguments
    joy_device_arg = DeclareLaunchArgument(
        'joy_device',
        default_value='/dev/input/event5',
        description='Path to joystick device file'
    )

    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='false',
        description='Enable debug logging'
    )

    max_linear_arg = DeclareLaunchArgument(
        'max_linear_speed',
        default_value='1.0',
        description='Maximum linear velocity (m/s)'
    )

    max_angular_arg = DeclareLaunchArgument(
        'max_angular_speed',
        default_value='1.0',
        description='Maximum angular velocity (rad/s)'
    )

    # Joy node (subscribes to /dev/input/eventX, publishes /joy)
    joy_node = Node(
        package='joy',
        executable='joy_node',
        parameters=[{
            'device': LaunchConfiguration('joy_device'),
            'deadzone': 0.05,
        }]
    )

    # Teleop DS4 node (subscribes to /joy, publishes /cmd_vel)
    teleop_node = Node(
        package='teleop_DS4',
        executable='teleop_node',
        parameters=[{
            'max_linear_speed': LaunchConfiguration('max_linear_speed'),
            'max_angular_speed': LaunchConfiguration('max_angular_speed'),
            'deadzone': 0.05,
            'accel_limit': 0.5,
            'watchdog_timeout': 0.2,
            'debug': LaunchConfiguration('debug'),
        }],
        remappings=[
            ('joy', 'joy'),
            ('cmd_vel', 'cmd_vel'),
        ]
    )

    return LaunchDescription([
        joy_device_arg,
        debug_arg,
        max_linear_arg,
        max_angular_arg,
        joy_node,
        teleop_node,
    ])
