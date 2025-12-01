from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition


def generate_launch_description():

    # Declare launch arguments
    gripper = LaunchConfiguration("gripper")
    gui = LaunchConfiguration("gui")
    gz_args = LaunchConfiguration("gz_args")

    gripper_arg = DeclareLaunchArgument(
        "gripper",      
        default_value="",
        description='Name of the gripper attached to the arm (empty for no gripper).'
    )

    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Whether to run the Gazebo GUI.'
    )

    gaz_args_arg = DeclareLaunchArgument(
        "gz_args",      
        default_value="-r -v 2 empty.sdf",
        description='Arguments to pass to Gazebo.'
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
                "sim_gazebo:=true",
                " ",
            ]
        )
    }

    robot_controllers = PathJoinSubstitution(
        [FindPackageShare("link6_control"),
         "config", "kortex3_controllers.yaml"]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ros_gz_sim"),
                                  "launch", "gz_sim.launch.py"])
        ),
        launch_arguments={
            "gz_args": gz_args,
            "gui": gui,
        }.items(),
    )

    spawn = Node(
        package    = "ros_gz_sim",
        executable = "create",
        output     = "screen",
        arguments  = [
            "-name",  "link6",
            "-topic", "robot_description",
            "-allow_renaming", "true"
        ],
    )

    robot_state_publisher_node = Node(
        package    = "robot_state_publisher",
        executable = "robot_state_publisher",
        output     = "screen",
        parameters = [robot_description, {"use_sim_time": True}],
    )

    js_broadcaster = Node(
        package    = "controller_manager",
        executable = "spawner",
        parameters = [{"use_sim_time": True}],
        arguments  = ["joint_state_broadcaster", "--controller-manager", "/controller_manager",  "--ros-args",
                      "--log-level", "WARN"],
        output     = "screen",
    )

    velocity_controller = Node(
        package    = "controller_manager",
        executable = "spawner",
        parameters = [{"use_sim_time": True}],
        arguments  = [
            "joint_velocity_controller",
            "--param-file", robot_controllers,
            "--controller-manager", "/controller_manager",
            "--inactive"
        ],
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

    motion_control_handle_spawner = Node(
        package="controller_manager",
        executable="spawner",
        parameters = [{"use_sim_time": True}],
        arguments=["motion_control_handle", "--inactive", "--controller-manager", "/controller_manager"],
        output="screen",
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

    load_js_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action = spawn,
            on_exit       = [js_broadcaster],
        )
    )

    load_controllers_after_js = RegisterEventHandler(
        OnProcessExit(
            target_action = js_broadcaster,
            on_exit=[
                velocity_controller,
                joint_trajectory_controller, 
                cartesian_motion_controller_spawner,
                motion_control_handle_spawner,
                robot_hand_controller_spawner,
            ],
        )
    )

    clock_bridge = Node(
        package    = "ros_gz_bridge",
        executable = "parameter_bridge",
        arguments  = ["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output     = "screen",
        parameters = [{"use_sim_time": True}],
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(gripper_arg)
    ld.add_action(gui_arg)
    ld.add_action(gaz_args_arg)

    ld.add_action(gazebo)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(spawn)
    ld.add_action(load_js_after_spawn)
    ld.add_action(load_controllers_after_js) 
    ld.add_action(clock_bridge)
    ld.add_action(topic_relay)

    return ld