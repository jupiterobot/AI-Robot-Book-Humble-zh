from launch import actions
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            namespace= "turtlesim1", package='turtlesim', executable='turtlesim_node', output='screen',  
            remappings=[('/cmd_vel', '/kame/cmd_vel')]),
        Node(
            namespace= "turtlesim2", package='turtlesim', executable='turtlesim_node', output='screen', 
            parameters=[{"background_b": 200}, {"background_g": 200},{"background_r": 200}]),
        Node(
            prefix='xterm -e', namespace= "turtlesim1", package='turtlesim', executable='turtle_teleop_key', output='screen',
            on_exit=actions.Shutdown()),
        Node(
            prefix='xterm -e', namespace= "turtlesim2", package='turtlesim', executable='turtle_teleop_key', output='screen')
    ])
