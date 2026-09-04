# SPDX-License-Identifier: GPL-3.0

import os
from glob import glob

from setuptools import find_packages, setup


package_name = "roboparty_dexhand_ros"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name,
         ["package.xml", "README.md", "CHANGELOG.md"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="RoboParty",
    maintainer_email="ZhihaoLiu_hit@163.com",
    description="ROS 2 device nodes for the roboparty_dexhand driver.",
    license="GPL-3.0",
    entry_points={
        "console_scripts": [
            "dexhand_node = roboparty_dexhand_ros.dexhand_node:main",
        ],
    },
)
