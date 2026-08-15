import os
from glob import glob

from setuptools import find_packages, setup

package_name = "ball_dropping_detection"

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
    maintainer="mavlab",
    maintainer_email="mavlab@example.com",
    description="Combined front and bottom camera drum detection for ball dropping task.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "front_drum_detector = ball_dropping_detection.front_drum_detector:main",
            "bottom_drum_detector = ball_dropping_detection.bottom_drum_detector:main",
            "drum_detection_fusion = ball_dropping_detection.drum_detection_fusion:main",
        ],
    },
)

