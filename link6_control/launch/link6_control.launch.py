import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression

from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    gripper = LaunchConfiguration("gripper")
    use_sim_time = LaunchConfiguration("use_sim_time")

    gripper_arg = DeclareLaunchArgument(
        "gripper",
        default_value="",
        description="Name of the gripper attached to the arm (empty for no gripper).",
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", 
        default_value='False',
        description="Wether to use simulation time.",
    )

    # Create the launch actions
    controller_config = os.path.join(
        get_package_share_directory("link6_control"),
        "config",
        "kortex3_controllers.yaml",
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controller_config],
        output="screen",
        arguments=["--ros-args", "--log-level", "WARN"],
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
    )

    js_broadcaster_spawner = Node(
        package    = "controller_manager",
        executable = "spawner",
        parameters = [{"use_sim_time": use_sim_time}],
        arguments  = ["joint_state_broadcaster", "--controller-manager", "/controller_manager",  "--ros-args",
                      "--log-level", "WARN"],
        output     = "screen",
    )

    joint_velocity_controller_spawner = Node(
        package    = "controller_manager",
        executable = "spawner",
        parameters = [{"use_sim_time": use_sim_time}],
        arguments  = [
            "joint_velocity_controller",
            "--controller-manager", "/controller_manager",
            "--inactive"
        ],
        output     = "screen",
    )

    joint_trajectory_controller_spawner = Node(
        package    = "controller_manager",
        executable = "spawner",
        parameters = [{"use_sim_time": use_sim_time}],
        arguments  = [
            "joint_trajectory_controller",
            "--controller-manager", "/controller_manager"
        ],
        output     = "screen",
    )

    cartesian_motion_controller_spawner = Node(
        package    = "controller_manager",
        executable = "spawner",
        parameters = [{"use_sim_time": use_sim_time}],
        arguments=[
            "cartesian_motion_controller", 
            "--controller-manager", 
            "/controller_manager",
            "--inactive"],
        output     = "screen",
    )

    motion_control_handle_spawner = Node(
        package="controller_manager",
        executable="spawner",
        parameters = [{"use_sim_time": use_sim_time}],
        arguments=["motion_control_handle", "--inactive", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=[
            "robotiq_gripper_controller",
            "--activate",
            "--controller-manager",
            "/controller_manager",
        ],
        condition=IfCondition(PythonExpression(["'", gripper, "' != ''"])),
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

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(gripper_arg)
    ld.add_action(use_sim_time_arg)

    ld.add_action(controller_manager)
    ld.add_action(js_broadcaster_spawner)
    ld.add_action(joint_velocity_controller_spawner)
    ld.add_action(joint_trajectory_controller_spawner)
    ld.add_action(cartesian_motion_controller_spawner)
    ld.add_action(motion_control_handle_spawner)
    ld.add_action(topic_relay)

    if gripper != "":
        ld.add_action(gripper_controller_spawner)

    return ld
