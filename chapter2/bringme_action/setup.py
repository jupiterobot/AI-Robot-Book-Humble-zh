from setuptools import setup

package_name = 'bringme_action'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kosei Demura',
    maintainer_email='ai-robot-book@googlegroups.com',
    description='Bringme Action Server and Client package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bringme_action_server_node = bringme_action.bringme_action_server_node:main',
            'bringme_action_client_node = bringme_action.bringme_action_client_node:main',          
        ],
    },
)
