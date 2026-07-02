"""Launch RViz for Lightning-LM SLAM visualization."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("lightning")
    rviz_config = LaunchConfiguration("rviz_config")
    default_rviz_config = PathJoinSubstitution(
        [pkg_share, "config", "showbodypc.rviz"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "rviz_config",
                default_value=default_rviz_config,
                description="RViz config file.",
            ),
            ExecuteProcess(
                cmd=["rviz2", "-d", rviz_config],
                name="rviz2",
                output="screen",
            ),
        ]
    )
