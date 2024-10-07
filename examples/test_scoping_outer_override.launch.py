
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_scoped_include.actions import IncludeScopedLaunchDescription
from launch.launch_description_sources import (
    get_launch_description_from_python_launch_file,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import ThisLaunchFileDir, PathJoinSubstitution


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument("Inner", default_value="True"),
        IncludeScopedLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    ThisLaunchFileDir(),
                    "test_scoping_inner.launch.py",
                ])
            )
        ),
    ])


