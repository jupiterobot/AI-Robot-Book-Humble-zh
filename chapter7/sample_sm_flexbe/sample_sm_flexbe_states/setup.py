#!/usr/bin/env python

from glob import glob
from setuptools import setup
from setuptools import find_packages

PACKAGE_NAME = 'sample_sm_flexbe_states'

setup(
    name=PACKAGE_NAME,
    version='2.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + PACKAGE_NAME]),
        ('share/' + PACKAGE_NAME, ['package.xml']),
        ('share/' + PACKAGE_NAME + "/tests", glob('tests/*.test')),
        ('share/' + PACKAGE_NAME + "/launch", glob('tests/*.launch.py')),
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
            'eat_state = sample_sm_flexbe_states.eat_state',
            'search_state = sample_sm_flexbe_states.search_state',
            'grasp_state = sample_sm_flexbe_states.grasp_state',
        ],
    },
)
