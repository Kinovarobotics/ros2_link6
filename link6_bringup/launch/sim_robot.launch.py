from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition


def generate_launch_description():

    #Initialize Arguments
    gripper = LaunchConfiguration("gripper")
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    gui          = LaunchConfiguration("gui",          default="true")
    gz_args      = LaunchConfiguration("gz_args",      default="-r -v 2 empty.sdf")

    robot_description = {
        "robot_description": Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                PathJoinSubstitution(
                    [FindPackageShare("link6_description"), "urdf", "link6_gz.urdf.xacro"]
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
         "config", "kortex3_sim_controllers.yaml"]
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

    rsp = Node(
        package    = "robot_state_publisher",
        executable = "robot_state_publisher",
        output     = "screen",
        parameters = [robot_description, {"use_sim_time": use_sim_time}],
    )


    js_broadcaster = Node(
        package    = "controller_manager",
        executable = "spawner",
        arguments  = ["joint_state_broadcaster", "--controller-manager", "/controller_manager",  "--ros-args",
                      "--log-level", "WARN"],
        output     = "screen",
    )


    velocity_controller = Node(
        package    = "controller_manager",
        executable = "spawner",
        arguments  = [
            "joint_velocity_controller",
            "--param-file", robot_controllers,
            "--controller-manager", "/controller_manager",
            "--inactive"
        ],
        output     = "screen",
    )

    cartesian_motion_controller_spawner = Node(
        package    = "controller_manager",
        executable = "spawner",
        arguments=["cartesian_motion_controller", "--controller-manager", "/controller_manager"],
        output     = "screen",
    )

    motion_control_handle_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["motion_control_handle", "--inactive", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    robot_hand_controller_spawner = Node(
    package="controller_manager",
    executable="spawner",
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
        parameters=[{"use_sim_time": use_sim_time}], 
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
        parameters = [{"use_sim_time": use_sim_time}],
    )


    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument("use_sim_time", default_value="true"))
    ld.add_action(DeclareLaunchArgument("gui",          default_value="true"))
    ld.add_action(DeclareLaunchArgument("gz_args",      default_value="-r -v 2 empty.sdf"))
    ld.add_action(DeclareLaunchArgument("gripper", default_value=""))

    ld.add_action(gazebo)
    ld.add_action(rsp)
    ld.add_action(spawn)
    ld.add_action(load_js_after_spawn)
    ld.add_action(load_controllers_after_js) 
    ld.add_action(clock_bridge)


    ld.add_action(topic_relay)

    return ld