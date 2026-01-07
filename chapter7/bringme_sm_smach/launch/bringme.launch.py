from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 想要启动的节点
        Node(
            package='bringme_sm_smach',
            executable='bringme_sm_smach',
        ),

        Node(
            package='pseudo_node_service',
            executable='manipulation_node',
        ),

        Node(
            package='pseudo_node_service',
            executable='navigation_node',
        ),

        Node(
            package='pseudo_node_service',
            executable='vision_node',
        ),

        Node(
            package='pseudo_node_service',
            executable='voice_node',
        )
    ]
)
