import os
from glob import glob

from setuptools import find_packages, setup

package_name = "abb_egm_controller"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="delta",
    maintainer_email="daniel.ruan@princeton.edu",
    description="ROS2 package for ABB EGM controller",
    license="MIT",
    entry_points={
        "console_scripts": ["egm = abb_egm_controller.egm_controller:main"],
    },
)
