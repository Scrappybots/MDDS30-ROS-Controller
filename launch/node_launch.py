from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments - Default port for Jetson Orin Nano
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyTHS0',
        description='Serial port for MDDS30 controller (Jetson Orin Nano default: /dev/ttyTHS0)'
    )

    # Create the node
    mdds30_node = Node(
        package='mdds30_controller',
        executable='mdds30_controller',
        name='mdds30_controller',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('port')
        }]
    )

    return LaunchDescription([
        port_arg,
        mdds30_node
    ])
