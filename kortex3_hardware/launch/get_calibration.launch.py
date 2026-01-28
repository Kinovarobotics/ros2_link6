"""
Launch file to download robot calibration and generate YAML file.

This launch file connects to a Kinova Link6 robot, downloads its calibration data,
and generates a YAML calibration file for use with the dynamic calibration system.

Usage:
    ros2 launch kortex3_hardware get_calibration.launch.py robot_ip:=192.168.1.10 \
        calibration_dir:="src/link6_description/config"
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            description="IP address of the robot (e.g., 192.168.1.10)",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "port",
            default_value="1883",
            description="MQTT port for robot connection (default: 1883)",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "username",
            default_value="admin",
            description="Robot username (default: admin)",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "password",
            default_value="admin",
            description="Robot password (default: admin)",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "output_file",
            default_value="calibration.yaml",
            description="Output YAML filename (default: calibration.yaml)",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "calibration_dir",
            description="Path to directory where calibration files will be stored "
                        "(e.g., src/link6_description/config)",
        )
    )

    # Create ROS node for calibration download
    calibration_node = Node(
        package='kortex3_hardware',
        executable='get_calibration_node',
        name='calibration_download',
        output='screen',
        parameters=[{
            'robot_ip': LaunchConfiguration('robot_ip'),
            'port': LaunchConfiguration('port'),
            'username': LaunchConfiguration('username'),
            'password': LaunchConfiguration('password'),
            'output_file': LaunchConfiguration('output_file'),
            'calibration_dir': LaunchConfiguration('calibration_dir'),
        }]
    )

    return LaunchDescription(
        declared_arguments + [
            calibration_node
        ]
    )
