from setuptools import setup

package_name = 'speech_topic'

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
    description='ROS2 package for speech recognition and speech synthesis',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'recognition = speech_topic.recognition:main',
            'synthesys = speech_topic.synthesys:main',
            'synthesys_mpg123 = speech_topic.synthesys_mpg123:main'
        ],
    },
)
