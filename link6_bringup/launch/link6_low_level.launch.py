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
)
from launch_ros.substitutions import FindPackageShare

_GRIPPER_JOINT_NAMES = {
    "robotiq_2f_85": "robotiq_85_left_knuckle_joint",
    "robotiq_2f_140": "finger_joint",
}

def launch_setup(context, *args, **kwargs):
    # Declare launch arguments
    gripper = LaunchConfiguration("gripper")
    gripper_joint_name = LaunchConfiguration("gripper_joint_name")
    use_internal_bus_gripper_comm = LaunchConfiguration("use_internal_bus_gripper_comm")
    calibration_file = LaunchConfiguration("calibration_file")
    robot_ip = LaunchConfiguration("robot_ip")
    username = LaunchConfiguration("username")
    password = LaunchConfiguration("password")
    operating_mode = LaunchConfiguration("operating_mode")
    safety_mode = LaunchConfiguration("safety_mode")

    gripper_str = gripper.perform(context)
    gripper_joint_name_str = gripper_joint_name.perform(context)
    if not gripper_joint_name_str:
        gripper_joint_name_str = _GRIPPER_JOINT_NAMES.get(gripper_str, "")

    robot_description = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("link6_description"), "urdf", "link6.urdf.xacro"]
            ),
            " ",
            "gripper:=",
            gripper_str,
            " ",
            "gripper_joint_name:=",
            gripper_joint_name_str,
            " ",
            "use_internal_bus_gripper_comm:=",
            use_internal_bus_gripper_comm,
            " ",
            "calibration_file:=",
            calibration_file,
            " ",
            "robot_ip:=",
            robot_ip,
            " ",
            "username:=",
            username,
            " ",
            "password:=",
            password,
            " ",
            "low_level_mode:=true",
            " ",
            "operating_mode:=",
            operating_mode,
            " ",
            "safety_mode:=",
            safety_mode,
            " ",
        ]
    )
    robot_description = {'robot_description': robot_description}

    controller_config = os.path.join(
        get_package_share_directory('link6_control'), 
        'config',
        'link6_controllers_low_level.yaml'
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    static_transform_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_base_link',
        output='screen',
        arguments=['--frame-id', 'world', '--child-frame-id', 'base_link'],
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controller_config],
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
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

    joint_trajectory_controller_spawner = Node(
        package    = "controller_manager",
        executable = "spawner",
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

    twist_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "twist_controller",
            "--inactive",
            "--controller-manager", "/controller_manager"
        ],
    )

    robotiq_gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "robotiq_gripper_controller", 
            "--activate", 
            "--controller-manager", "/controller_manager"
        ]
    )

    nodes_to_start = [
        robot_state_publisher_node,
        static_transform_publisher,
        controller_manager,
        joint_state_broadcaster_spawner,
        joint_trajectory_controller_spawner,
        joint_velocity_controller_spawner,
        twist_controller_spawner,
    ]
    if gripper.perform(context) != "":
        nodes_to_start.append(robotiq_gripper_controller_spawner)
    return nodes_to_start

def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "gripper",
            default_value="",
            description='Name of the gripper attached to the arm (empty for no gripper).',
            choices=["", "robotiq_2f_85", "robotiq_2f_140"],
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gripper_joint_name",
            default_value="",
            description=(
                "Name of the actuated joint in the gripper to be used by the controller. "
                "Defaults to the standard joint name for the selected gripper if left empty."
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_internal_bus_gripper_comm",
            default_value="true",
            description="Use internal bus for gripper communication?",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "calibration_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("link6_description"), "config", "default_calibration.yaml"]
            ),
            description="Path to robot-specific calibration YAML file (default: default_calibration.yaml)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            default_value="192.168.1.10",
            description="IP address of the robotic arm",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "username",
            default_value="admin",
            description="username to start a session to interact with the robot",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "password",
            default_value="admin",
            description="password to start a session to interact with the robot",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "operating_mode",
            default_value="auto",
            description="Operating mode used when the low-level position controller is active.",
            choices=["hold_to_run", "auto"],
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_mode",
            default_value="reduced",
            description="Safety system mode used when the low-level position controller is active.",
            choices=["reduced", "normal"],
        )
    )
    return LaunchDescription(declared_arguments+[OpaqueFunction(function=launch_setup)])