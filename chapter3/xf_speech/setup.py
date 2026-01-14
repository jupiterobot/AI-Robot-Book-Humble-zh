from setuptools import find_packages, setup

package_name = 'xf_speech'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'xf_topic_tts = xf_speech.xf_topic_tts:main',
            'xf_topic_iat = xf_speech.xf_topic_iat:main',
            'xf_recognition_server = xf_speech.xf_recognition_server:main',
            'xf_recognition_client = xf_speech.xf_recognition_client:main',
            'xf_synthesis_server = xf_speech.xf_synthesis_server:main',
            'xf_synthesis_client = xf_speech.xf_synthesis_client:main',
            'xf_speech_client = xf_speech.xf_speech_client:main',
        ],
    },
)
