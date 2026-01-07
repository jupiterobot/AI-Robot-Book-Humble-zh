import os
from glob import glob
from setuptools import setup

package_name = 'tf_examples'

setup(
    name=package_name,
    version='2.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='MASUTANI Yasuhiro',
    maintainer_email='ai-robot-book@googlegroups.com',
    description='TF examples for AI Robot Book',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'satellite_broadcaster = tf_examples.satellite_broadcaster:main',
            'planet_broadcaster = tf_examples.planet_broadcaster:main',
            'satellite_listener = tf_examples.satellite_listener:main',
            'dummy_sensor_publisher = tf_examples.dummy_sensor_publisher:main',
            'dummy_sensor_subscriber = tf_examples.dummy_sensor_subscriber:main',
        ],
    },
)
