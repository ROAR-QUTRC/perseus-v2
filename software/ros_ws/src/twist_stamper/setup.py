from setuptools import find_packages, setup

package_name = "twist_stamper"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="DingoOz",
    maintainer_email="nigel.hungerfordsymes@gmail.com",
    description="Republishes an unstamped Twist as a TwistStamped (adds header/timestamp)",
    license="MIT",
    entry_points={
        "console_scripts": [
            "twist_stamper = twist_stamper.twist_stamper:main",
        ],
    },
)
