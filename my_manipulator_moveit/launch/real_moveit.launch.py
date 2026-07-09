from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    serial_port = LaunchConfiguration("serial_port")
    baud_rate = LaunchConfiguration("baud_rate")
    write_rate_hz = LaunchConfiguration("write_rate_hz")
    dry_run = LaunchConfiguration("dry_run")

    moveit_config = (
        MoveItConfigsBuilder("manipulator", package_name="my_manipulator_moveit")
        .trajectory_execution(moveit_manage_controllers=False)
        .to_moveit_configs()
    )

    real_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("my_manipulator"),
                "launch",
                "real_robot.launch.py",
            ])
        ),
        launch_arguments={
            "serial_port": serial_port,
            "baud_rate": baud_rate,
            "write_rate_hz": write_rate_hz,
            "dry_run": dry_run,
        }.items(),
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                "use_sim_time": False,
                "publish_robot_description_semantic": True,
                "allow_trajectory_execution": True,
                "publish_planning_scene": True,
                "publish_geometry_updates": True,
                "publish_state_updates": True,
                "publish_transforms_updates": True,
                "monitor_dynamics": False,
            },
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", str(moveit_config.package_path / "config/moveit.rviz")],
        parameters=[
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            {"use_sim_time": False},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "serial_port",
            default_value="/dev/ttyUSB0",
            description="Arduino serial device",
        ),
        DeclareLaunchArgument(
            "baud_rate",
            default_value="9600",
            description="Arduino serial baud rate",
        ),
        DeclareLaunchArgument(
            "write_rate_hz",
            default_value="20",
            description="Maximum serial command rate",
        ),
        DeclareLaunchArgument(
            "dry_run",
            default_value="false",
            description="If true, print Arduino CSV commands without opening the serial port",
        ),
        real_robot_launch,
        TimerAction(
            period=5.0,
            actions=[
                move_group_node,
                rviz_node,
            ],
        ),
    ])
