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
        DeclareLaunchArgument("Kaas", default_value="True"),
        IncludeScopedLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    ThisLaunchFileDir(),
                    "test_scoping_inner.launch.py",
                ])
            )
        ),
    ])


if __name__ == "__main__":
    import sys
    from launch import LaunchService

    argv = sys.argv[1:]
    ls = LaunchService(argv=argv)

    ld = generate_launch_description()
    ls.include_launch_description(ld)
    ls.run()
