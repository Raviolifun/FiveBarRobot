import os
from glob import glob
from setuptools import setup
from setuptools import find_packages

package_name = "five_bar_robot"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/rviz", glob("rviz/*.rviz")),
        (os.path.join("share", package_name), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="aditya",
    maintainer_email="apenumarti@ufl.edu",
    description="Package for controlling five bar robot",
    license="MIT",
    entry_points={
        "console_scripts": [
            "driver = five_bar_robot.odrive_driver:main",
            "pose_demo = five_bar_robot.pose_demo:main",
            "puck_detector = five_bar_robot.cam_sub:main",
        ],
    },
)
