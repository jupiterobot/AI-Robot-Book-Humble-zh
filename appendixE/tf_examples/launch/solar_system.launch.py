from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    rviz_config_file = get_package_share_directory(
        'tf_examples') + '/launch/solar_system.rviz'

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
    )

    pb = Node(
        package='tf_examples',
        executable='planet_broadcaster',
        arguments=[],
    )

    sb = Node(
        package='tf_examples',
        executable='satellite_broadcaster',
        arguments=[],
    )

    pb2 = Node(
        package='tf_examples',
        executable='planet_broadcaster',
        arguments=['planet2', '1', '8'],
    )

    sl = Node(
        package='tf_examples',
        executable='satellite_listener',
        arguments=['planet2'],
    )

    return LaunchDescription([rviz, pb, sb, pb2, sl])
