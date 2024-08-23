from setuptools import find_packages, setup
import os
from glob import glob

package_name = "force_torque_sensor"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.*")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Nikola Banovic",
    maintainer_email="nibanovic@gmail.com",
    description="calibration for robot tool",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "calibrate_tool_node = force_torque_sensor.calibrate_tool:main"
        ],
    },
)
