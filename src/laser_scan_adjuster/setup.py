from setuptools import find_packages, setup

package_name = "laser_scan_adjuster"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
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
        "ROS 2 LaserScan adapter that normalizes range-array length and "
        "republishes sensor-data QoS."
    ),
    license="All rights reserved",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "adjust_scan = laser_scan_adjuster.adjust_scan:main",
        ],
    },
)
