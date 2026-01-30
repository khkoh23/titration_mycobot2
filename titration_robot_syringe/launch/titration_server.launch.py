from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    delta_ph_server_node = Node(
        package='titration_robot_syringe',
        executable='delta_ph_server',
        output='screen',
    )

    derivative_ph_volume_server_node = Node(
        package='titration_robot_syringe',
        executable='derivative_ph_volume_server',
        output='screen',
    )

    syringe_server_node = Node(
        package='titration_robot_syringe',
        executable='syringe_server',
        output='screen',
    )
    
    return LaunchDescription([
        delta_ph_server_node,
        derivative_ph_volume_server_node,
        syringe_server_node,
    ])