from setuptools import find_packages, setup

package_name = "map_gui"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="matthias_the_british_duck",
    maintainer_email="matthias_the_british_duck@todo.todo",
    description="TODO: Package description",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "ExtractFeatures = map_gui.extract_features:main",
            "connection_comedy = map_gui.connection_comedy:main",
        ],
    },
)
