import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
)
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.substitutions import FindPackageShare

def launch_setup(context, *args, **kwargs):
    # Declare launch arguments
    gripper = LaunchConfiguration("gripper")
    robot_description = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("link6_description"), "urdf", "link6.urdf.xacro"]
            ),
            " ",
            "gripper:=", 
            gripper,
            " ",


        ]
    )


    robot_description = {'robot_description': robot_description}


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

    joint_trajectory_controller = Node(
        package    = "controller_manager",
        executable = "spawner",
        parameters = [{"use_sim_time": True}],
        arguments  = [
            "joint_trajectory_controller",
            "--param-file", controller_config,
            "--controller-manager", "/controller_manager"
        ],
        output     = "screen",
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

    robot_hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["robotiq_gripper_controller", "--activate", "--controller-manager", "/controller_manager"]
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

    nodes_to_start = [
        robot_state_publisher_node,
        tf,
        controller_manager,
        joint_state_broadcaster_spawner,
        cartesian_motion_controller_spawner,
        motion_control_handle_spawner,
        joint_trajectory_controller
        joint_velocity_controller_spawner,
        topic_relay,
    ]
    if gripper.perform(context) != "":
        nodes_to_start.append(robot_hand_controller_spawner)
    return nodes_to_start

def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "gripper",
            default_value="",
            description="Name of the gripper attached to the arm (empty for no gripper)",
        )
    )
    return LaunchDescription(declared_arguments+[OpaqueFunction(function=launch_setup)])