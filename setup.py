from setuptools import find_packages, setup

package_name = "launch_scoped_include"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    package_data={package_name: ['py.typed']},
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jap",
    maintainer_email="36795178+SuperJappie08@users.noreply.github.com",
    description="Extension to `launch` to include launchfiles in a scoped manner.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "launch.frontend.launch_extension": [
            "launch_scoped_include = launch_scoped_include",
        ],
    },
)
