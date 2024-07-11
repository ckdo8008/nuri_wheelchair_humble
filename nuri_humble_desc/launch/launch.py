# # ~/ros2_ws/src/nuri_humble_desc/launch/display.launch.py
# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
# from launch_ros.actions import Node
# from ament_index_python.packages import get_package_share_directory

# def generate_launch_description():
#     urdf_file_path = PathJoinSubstitution([
#         get_package_share_directory('nuri_humble_desc'),
#         'urdf',
#         'robot.urdf.xacro'
#     ])

#     return LaunchDescription([
#         DeclareLaunchArgument(
#             'urdf_file',
#             default_value=urdf_file_path,
#             description='URDF/XACRO file to describe the robot'
#         ),
#         Node(
#             package='robot_state_publisher',
#             executable='robot_state_publisher',
#             output='screen',
#             parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('urdf_file')])}]
#         )
#     ])
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="nuri_humble_desc",
            description="Description package with robot URDF/xacro files. Usually the argument \
        is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="robot.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="false",
            description="Start Rviz2 and Joint State Publisher gui automatically \
        with this launch file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed than also joint names in the controllers' configuration \
        have to be updated.",
        )
    )

    # Initialize Arguments
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    gui = LaunchConfiguration("gui")
    prefix = LaunchConfiguration("prefix")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("nuri_humble_desc"), "urdf", description_file]
            ),
            " ",
            "prefix:=",
            prefix,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        condition=IfCondition(gui),
    )
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    nodes = [
        joint_state_publisher_node,
        robot_state_publisher_node,
    ]

    return LaunchDescription(declared_arguments + nodes)
