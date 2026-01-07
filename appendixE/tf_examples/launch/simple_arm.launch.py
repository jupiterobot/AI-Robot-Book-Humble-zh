import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    urdf_file_name = 'simple_arm.urdf'
    urdf = os.path.join(
        get_package_share_directory('simple_arm_description'),
        'urdf',
        urdf_file_name)
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()
    params = {'robot_description': robot_desc}

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[params],
    )

    jsp = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
    )

    rviz_config_file = get_package_share_directory(
        'tf_examples') + '/launch/simple_arm.rviz'

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
    )

    stp = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '1', '--y', '0', '--z', '0.05',
            '--roll', '-1.57', '--pitch', '0', '--yaw', '-1.57',
            '--frame-id', 'link2', '--child-frame-id', 'dummy_sensor'],
    )

    dsp = Node(
        package='tf_examples',
        executable='dummy_sensor_publisher',
        arguments=[],
    )

    dss = Node(
        package='tf_examples',
        executable='dummy_sensor_subscriber',
        arguments=[],
    )

    return LaunchDescription([rsp, jsp, rviz, stp, dsp, dss])
