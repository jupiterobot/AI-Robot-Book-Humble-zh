#!/usr/bin/env python
from setuptools import setup

package_name = 'sample_sm_flexbe_behaviors'

setup(
    name=package_name,
    version='2.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Valentin Cardenas Keith',
    maintainer_email='ai-robot-book@googlegroups.com',
    description='ROS2 package for a simple state machine using FlexBE',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sample_behavior_sm = sample_sm_flexbe_behaviors.sample_behavior_sm',
        ],
    },
)
