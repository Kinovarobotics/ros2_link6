from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition


def generate_launch_description():

    # Declare launch arguments
    gripper = LaunchConfiguration("gripper")

    gripper_arg = DeclareLaunchArgument(
        "gripper",      
        default_value="",
        description='Name of the gripper attached to the arm (empty for no gripper).'
    )


    # Create the launch actions
    robot_description = {
        "robot_description": Command(
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
    }

    robot_controllers = PathJoinSubstitution(
        [FindPackageShare("link6_control"),
         "config", "kortex3_controllers.yaml"]
    )

    robot_state_publisher_node = Node(
        package    = "robot_state_publisher",
        executable = "robot_state_publisher",
        output     = "screen",
        parameters = [robot_description, {"use_sim_time": True}],
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="screen",
        arguments=[
            "--ros-args",
            "--log-level", "WARN"
        ],
    )

    js_broadcaster = Node(
        package    = "controller_manager",
        executable = "spawner",
        parameters = [{"use_sim_time": True}],
        arguments  = ["joint_state_broadcaster", "--controller-manager", "/controller_manager",  "--ros-args",
                      "--log-level", "WARN"],
        output     = "screen",
    )

    joint_trajectory_controller = Node(
        package    = "controller_manager",
        executable = "spawner",
        parameters = [{"use_sim_time": True}],
        arguments  = [
            "joint_trajectory_controller",
            "--param-file", robot_controllers,
            "--controller-manager", "/controller_manager"
        ],
        output     = "screen",
    )

    cartesian_motion_controller_spawner = Node(
        package    = "controller_manager",
        executable = "spawner",
        parameters = [{"use_sim_time": True}],
        arguments=[
            "cartesian_motion_controller", 
            "--controller-manager", 
            "/controller_manager",
            "--inactive"],
        output     = "screen",
    )

    robot_hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        parameters = [{"use_sim_time": True}],
        arguments=["robotiq_gripper_controller", "--activate", "--controller-manager", "/controller_manager"],
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
        parameters=[{"use_sim_time": True}], 
    )

    load_controllers_after_js = RegisterEventHandler(
        OnProcessExit(
            target_action = js_broadcaster,
            on_exit=[
                joint_trajectory_controller, 
                # cartesian_motion_controller_spawner,
                robot_hand_controller_spawner,
            ],
        )
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(gripper_arg)

    ld.add_action(robot_state_publisher_node)
    ld.add_action(controller_manager)
    ld.add_action(js_broadcaster)
    ld.add_action(load_controllers_after_js)
    ld.add_action(topic_relay)

    return ld