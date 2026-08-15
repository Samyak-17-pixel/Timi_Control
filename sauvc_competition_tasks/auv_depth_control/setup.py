import os
from glob import glob

from setuptools import find_packages, setup

package_name = "auv_depth_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="aatmaj",
    maintainer_email="na22b018@smail.iitm.ac.in",
    description="Depth hold PID and cmd_vel merge for 3D thruster mixing.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "depth_controller = auv_depth_control.depth_controller_node:main",
            "cmd_vel_3d_merge = auv_depth_control.cmd_vel_merge:main",
        ],
    },
)
