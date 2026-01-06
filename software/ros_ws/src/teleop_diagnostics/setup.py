from setuptools import find_packages, setup
import os
from glob import glob

package_name = "teleop_diagnostics"

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
    maintainer="ROAR Team",
    maintainer_email="roar@qut.edu.au",
    description="TUI diagnostic tool for teleoperation debugging",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "teleop_tui = teleop_diagnostics.teleop_tui:main",
        ],
    },
)
