from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gluemaster',
            namespace='gluemaster1',
            executable='gm_encoder_node',
            output='screen',
            name='gm_encoder'
        ),
        Node(
            package='gluemaster',
            namespace='gluemaster1',
            executable='gm_machine_node',
            output='screen',
            name='gm_machine'
        ),
        Node(
            package='gluemaster',
            namespace='gluemaster1',
            executable='gm_rec_node',
            output='screen',
            name='gm_rec'
        ),
        Node(
            package='gluemaster',
            namespace='gluemaster1',
            executable='gm_ui_node',
            output='screen',
            name='gm_ui'
        ),

    ])