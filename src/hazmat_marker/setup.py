from setuptools import setup

package_name = "hazmat_marker"

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
        "ROS 2 mission-annotation package for publishing detected HAZMAT "
        "locations as RViz map markers."
    ),
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "hazmat_marker = hazmat_marker.hazmat_marker:main",
        ],
    },
)
