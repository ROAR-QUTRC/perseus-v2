from setuptools import find_packages, setup

package_name = "input_devices"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="james",
    maintainer_email="59348282+RandomSpaceship@users.noreply.github.com",
    description="Nodes which publish input device data to topics",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "generic_controller = input_devices.generic_controller:main",
            "xbox_controller = input_devices.xbox_controller:main",
        ],
    },
)
