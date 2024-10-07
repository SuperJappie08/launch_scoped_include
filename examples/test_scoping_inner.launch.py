from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import TextSubstitution, LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument("Inner", default_value="False"),
        LogInfo(msg=["Inner", LaunchConfiguration("Inner")]),
    ])
