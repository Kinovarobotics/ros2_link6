import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():




    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("link6_description"), "urdf", "link6.xacro"]
            ),
            " ",

        ]
    )


    robot_description = {'robot_description': robot_description_content}


    controller_config = os.path.join(
        get_package_share_directory('link6_control'), 
        'config',
        'kortex3_controllers.yaml'
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )


    tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_base_link',
        output='screen',
        arguments=['0','0','0','0','0','0','world','base_link'],
    )



    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controller_config],
        output="screen",
        arguments=[
            "--ros-args",
            "--log-level", "WARN"
        ],
    )


    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
            "--ros-args", "--log-level", "ERROR"
        ],
        output="screen",
    )


    joint_velocity_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_velocity_controller",
            "--inactive",
            "--controller-manager", "/controller_manager"
        ],
    )

    cartesian_motion_controller_spawner = Node(
            package    = "controller_manager",
            executable = "spawner",
            arguments=["cartesian_motion_controller", "--activate", "--controller-manager-timeout", "300"],
            output     = "screen",
        )

    motion_control_handle_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["motion_control_handle",  "--inactive", "--controller-manager-timeout", "300"],
    )

    topic_relay = Node(
        package="topic_tools",
        executable="relay",
        arguments=[
            "/motion_control_handle/target_frame",
            "/cartesian_motion_controller/target_frame",
        ],
        output="screen",
    )


    return LaunchDescription([
        robot_state_publisher_node,
        tf,
        controller_manager,
        joint_state_broadcaster_spawner,
        cartesian_motion_controller_spawner,
        motion_control_handle_spawner,
        joint_velocity_controller_spawner,
        topic_relay,
    ])