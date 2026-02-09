from setuptools import setup, find_packages

setup(
    name="roar-ml",
    version="0.0.1",
    packages=find_packages(where="src"),
    package_dir={"": "src"},
    install_requires=[
        "torch",
    ],
    python_requires=">=3.8",
)
