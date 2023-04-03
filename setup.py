from setuptools import setup

package_name = 'anrs'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
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
