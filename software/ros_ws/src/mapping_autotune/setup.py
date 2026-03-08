from setuptools import find_packages, setup
import os
from glob import glob

package_name = "mapping_autotune"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "scripts"), glob("scripts/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ROAR Team",
    maintainer_email="roar@qut.edu.au",
    description="Automated SLAM parameter tuning for mapping quality optimization",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "autotune_node = mapping_autotune.autotune_node:main",
            "autotune_setup = mapping_autotune.setup_tui:main",
            "review_tui = mapping_autotune.review_tui:main",
            "imu_filter_node = mapping_autotune.imu_filter_node:main",
            "export_report = mapping_autotune.db_manager:export_report_cli",
            "calibrate_odom = mapping_autotune.calibration_node:main",
        ],
    },
)
