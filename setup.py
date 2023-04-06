import os
from setuptools import setup, find_packages

package_name = 'anrs'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    package_data={'anrs': ['launch/*.launch', 'drivers/*']},
    setup_requires=['setuptools'],
    install_requires=['setuptools', 'python-rpi.gpio'],
    maintainer='ethan moore',
    maintainer_email='ethanmoore1414@gmail.com',
    description='Automated Neural Recording System ROS2 package',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stepper = anrs.stepper:main',
        ],
    },
)
