from setuptools import setup

package_name = 'pseudo_node_service'

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
    maintainer='Masaki Ito',
    maintainer_email='ai-robot-book@googlegroups.com',
    description='Pseudo nodes for the state machine of Bring Me task',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'manipulation_node = pseudo_node_service.manipulation_node:main',
            'navigation_node = pseudo_node_service.navigation_node:main',
            'vision_node = pseudo_node_service.vision_node:main',
            'voice_node = pseudo_node_service.voice_node:main'
        ],
    },
)
