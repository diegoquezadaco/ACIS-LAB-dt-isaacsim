from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='kinova_dt',
            executable='gui_joint_control',
            name='gui_joint_control',
            output='screen'
        ),
    ])  
