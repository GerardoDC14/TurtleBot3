from setuptools import setup

package_name = "turtlebot3_poi_navigation"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "PyYAML"],
    zip_safe=True,
    maintainer="Gerardo Escobar",
    maintainer_email="123440177+GerardoDC14@users.noreply.github.com",
    description=(
        "Interactive ROS 2 point-of-interest manager for storing named goals "
        "and dispatching Nav2 missions."
    ),
    license="All rights reserved",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "poi_manager = turtlebot3_poi_navigation.poi_manager:main",
        ],
    },
)
