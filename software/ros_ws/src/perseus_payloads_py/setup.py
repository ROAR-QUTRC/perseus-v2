from setuptools import find_packages, setup

package_name = "perseus_payloads_py"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "pyserial"],
    zip_safe=True,
    maintainer="conner",
    maintainer_email="connerjs345@gmail.com",
    description="Package for python nodes belonging to payloads.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "centrifuge_driver = perseus_payloads_py.centrifuge_driver:main",
            "ilmenite_spectral_sensor = perseus_payloads_py.ilmenite_spectral_sensor:main",
            "ilmenite_ml = perseus_payloads_py.ilmenite_ml:main",
        ],
    },
)
