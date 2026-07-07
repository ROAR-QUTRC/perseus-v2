import os
from glob import glob

from setuptools import find_packages, setup

package_name = "perseus_isaac_relay"

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
    maintainer="DingoOz",
    maintainer_email="nigel.hungerfordsymes@gmail.com",
    description="Isaac ROS -> standard-message relay for zenoh bridging.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "relay = perseus_isaac_relay.relay_node:main",
        ],
    },
)
