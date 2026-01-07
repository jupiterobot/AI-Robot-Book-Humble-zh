import os
from glob import glob
from setuptools import setup

package_name = 'pseudo_node_action'

setup(
    name=package_name,
    version='2.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Valentin Cardenas Keith',
    maintainer_email='ai-robot-book@googlegroups.com',
    description='Pseudo action nodes for the state machine of Bring Me task',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'manipulation_node = pseudo_node_action.manipulation_node:main',
            'navigation_node = pseudo_node_action.navigation_node:main',
            'vision_node = pseudo_node_action.vision_node:main',
            'voice_node = pseudo_node_action.voice_node:main'
        ],
    },
)
