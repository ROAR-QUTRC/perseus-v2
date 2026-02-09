from setuptools import setup

setup(
    name="roar_ml",
    version="0.0.1",
    py_modules=["roar_ml"],
    install_requires=[
        "torch",
    ],
    python_requires=">=3.8",
    entry_points={
        "console_scripts": [
            "roar_ml=roar_ml:main",
        ],
    },
)
