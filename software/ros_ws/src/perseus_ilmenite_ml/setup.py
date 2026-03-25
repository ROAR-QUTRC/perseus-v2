from setuptools import find_packages, setup

package_name = "perseus_ilmenite_ml"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "torch"],
    zip_safe=True,
    maintainer="matthias_the_british_duck",
    maintainer_email="175209784+MatthiasTheBritishDuck@users.noreply.github.com",
    description="TODO: Package description",
    license="MIT",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "ilmenite_ml = perseus_ilmenite_ml.ilmenite_controller:main",
            "test_ml = perseus_ilmenite_ml.test:main",
        ],
    },
)
