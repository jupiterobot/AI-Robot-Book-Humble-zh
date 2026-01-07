from setuptools import find_packages, setup

package_name = 'airobot_action'

setup(
    name=package_name,
    version='2.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='MASUTANI Yasuhiro',
    maintainer_email='ai-robot-book@googlegroups.com',
    description='Action examples for AI Robot Book',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'new_bringme_action_server_node = airobot_action.new_bringme_action_server_node:main',
            'test_client = airobot_action.test_client:main',
        ],
    },
)
