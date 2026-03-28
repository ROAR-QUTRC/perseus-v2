from setuptools import find_packages, setup
import os
from glob import glob

package_name = "perseus_lite_drifting"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "audio"), glob("audio/*.mp3")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Nigel_H-S",
    maintainer_email="1388693+DingoOz@users.noreply.github.com",
    description="Plays a drift sound effect when Perseus Lite is drifting",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "drift_detector_node = perseus_lite_drifting.drift_detector_node:main",
        ],
    },
)
