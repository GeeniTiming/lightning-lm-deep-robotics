"""Launch online Lightning-LM SLAM with optional RViz visualization."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    RegisterEventHandler,
    Shutdown,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackagePrefix, FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("lightning")
    pkg_prefix = FindPackagePrefix("lightning")

    config = LaunchConfiguration("config")
    rviz = LaunchConfiguration("rviz")
    rviz_config = LaunchConfiguration("rviz_config")

    default_config = PathJoinSubstitution(
        [pkg_share, "config", "default_deep_robotics.yaml"]
    )
    default_rviz_config = PathJoinSubstitution(
        [pkg_share, "config", "showbodypc.rviz"]
    )
    slam_executable = PathJoinSubstitution(
        [pkg_prefix, "lib", "lightning", "run_slam_online"]
    )

    slam = ExecuteProcess(
        cmd=[slam_executable, "--config", config],
        name="lightning_slam_online",
        output="screen",
    )

    rviz_process = ExecuteProcess(
        cmd=["rviz2", "-d", rviz_config],
        name="rviz2",
        output="screen",
        condition=IfCondition(rviz),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config",
                default_value=default_config,
                description="YAML config passed to run_slam_online.",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="true",
                description="Start RViz together with online SLAM.",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=default_rviz_config,
                description="RViz config file.",
            ),
            slam,
            TimerAction(period=2.0, actions=[rviz_process]),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=slam,
                    on_exit=[
                        Shutdown(reason="lightning_slam_online exited"),
                    ],
                )
            ),
        ]
    )
