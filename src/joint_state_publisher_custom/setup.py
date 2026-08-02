from setuptools import setup

package_name = "joint_state_publisher_custom"

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
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Gerardo Escobar",
    maintainer_email="123440177+GerardoDC14@users.noreply.github.com",
    description=(
        "Custom ROS 2 joint-state publisher for visualizing the parallel "
        "mechanism attached to the TurtleBot3 platform."
    ),
    license="All rights reserved",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "joint_state_publisher = "
            "joint_state_publisher_custom.joint_state_publisher:main",
        ],
    },
)
