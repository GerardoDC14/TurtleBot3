from glob import glob
import os

from setuptools import setup

package_name = "turtlebot3_explore"

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
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Gerardo Escobar",
    maintainer_email="123440177+GerardoDC14@users.noreply.github.com",
    description=(
        "Python frontier-exploration mission node using occupancy-grid processing, "
        "TF and Nav2 goal execution."
    ),
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "explore = turtlebot3_explore.explore:main",
        ],
    },
)
