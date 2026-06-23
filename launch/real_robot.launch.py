from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false", # 실제 로봇은 시뮬레이션 시간을 사용하지 않습니다.
            description="Use simulation (Gazebo) clock if true",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "serial_port",
            default_value="/dev/ttyUSB0",
            description="Arduino serial device",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "baud_rate",
            default_value="115200",
            description="Arduino serial baud rate",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "write_rate_hz",
            default_value="20",
            description="Maximum serial command rate",
        )
    )

    # Initialize Arguments
    use_sim_time = LaunchConfiguration("use_sim_time")
    serial_port = LaunchConfiguration("serial_port")
    baud_rate = LaunchConfiguration("baud_rate")
    write_rate_hz = LaunchConfiguration("write_rate_hz")

    # Get the package path
    pkg_manipulator_path = FindPackageShare('manipulator')

    # 실제 로봇용 URDF 파일 경로 설정
    xacro_file = PathJoinSubstitution(
        [pkg_manipulator_path, "urdf", "manipulator_real.urdf.xacro"]
    )
    
    # xacro 명령어를 사용하여 URDF 생성
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            xacro_file,
            " serial_port:=",
            serial_port,
            " baud_rate:=",
            baud_rate,
            " write_rate_hz:=",
            write_rate_hz,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # ros2_control 노드: 하드웨어 인터페이스와 컨트롤러 매니저를 로드합니다.
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, 
                    PathJoinSubstitution([pkg_manipulator_path, "config", "my_controllers.yaml"])],
        output="screen",
    )

    # Robot state publisher: TF 정보를 발행합니다.
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )

    # Joint state broadcaster spawner: 컨트롤러 매니저가 활성화된 후 실행됩니다.
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # Joint trajectory controller spawner: 컨트롤러 매니저가 활성화된 후 실행됩니다.
    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "--controller-manager", "/controller_manager"],
    )

    return LaunchDescription(declared_arguments + [
        ros2_control_node,
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        joint_trajectory_controller_spawner,
    ])
