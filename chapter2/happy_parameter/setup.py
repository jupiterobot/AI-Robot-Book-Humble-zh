from setuptools import find_packages, setup

package_name = 'happy_parameter'

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
    maintainer='ubuntu',
    maintainer_email='ai-robot-book@googlegroups.com',
    description='A happy parameter package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'happy_parameter_server_node = happy_parameter.happy_parameter_server_node:main',
            'happy_parameter_client_node = happy_parameter.happy_parameter_client_node:main',
        ],
    },
)
