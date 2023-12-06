from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution


def generate_launch_description():

    turtlebot3_bringup_pkg_prefix = get_package_share_directory("turtlebot3_bringup")

    tbot3 = IncludeLaunchDescription(PythonLaunchDescriptionSource([turtlebot3_bringup_pkg_prefix,"/launch/robot.launch.py"]))
    return LaunchDescription([
        tbot3,
        Node(
            package="v4l2_camera",
            executable="v4l2_camera_node",
            name="v4l2_camera"
        )
    ])