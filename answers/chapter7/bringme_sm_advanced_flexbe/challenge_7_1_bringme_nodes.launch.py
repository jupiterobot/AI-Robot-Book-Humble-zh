from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 描述想要启动的节点
        Node(
            package='pseudo_node_action',
            executable='manipulation_node',
        ),

        Node(
            package='pseudo_node_action',
            executable='navigation_node',
        ),

        Node(
            package='pseudo_node_action',
            executable='vision_node',
        ),

        Node(
            package='speech_action',
            executable='speech_recognition_server',
        )
    ]
)
