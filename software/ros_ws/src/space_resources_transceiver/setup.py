import os
from glob import glob

from setuptools import find_packages, setup

package_name = "space_resources_transceiver"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.[pxy][yma]*")),
        ),
        (
            os.path.join("share", package_name, "config"),
            glob(os.path.join("config", "*.yaml")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ROAR Team",
    maintainer_email="roar@qut.edu.au",
    description="915MHz transceiver bridge for space resources end effector",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "transceiver_node = space_resources_transceiver.transceiver_node:main",
        ],
    },
)
