from setuptools import setup, find_packages

setup(
    name="roar_ml",
    version="0.0.1",
    packages=find_packages(where="src"),
    package_dir={"": "src"},
    install_requires=[
        "torch",
        "pyyaml",
    ],
    python_requires=">=3.8",
    entry_points={
        "console_scripts": [
            "roar_ml=roar_ml:main",
        ],
    },
)
