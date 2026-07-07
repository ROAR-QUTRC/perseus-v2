from setuptools import find_packages, setup

package_name = 'perseus_lite_tui'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    # Declared via extras_require (not the deprecated `tests_require=`, see
    # ERRORS.md 2026-07-03) so colcon selects its pytest runner instead of the
    # setup.py/unittest fallback, which finds no tests and fails with exit 5.
    extras_require={'test': ['pytest']},
    zip_safe=True,
    maintainer='DingoOz',
    maintainer_email='nigel.hungerfordsymes@gmail.com',
    description='Curses mission-control TUI for the perseus-lite software stack.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'perseus_tui = perseus_lite_tui.app:main',
        ],
    },
)
