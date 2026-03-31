from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():
    default_output_dir = os.path.join(os.environ.get('HOME', '/tmp'), 'titration_logs')

    log_rate_arg = DeclareLaunchArgument(
        'log_rate_hz',
        default_value='5.0',
        description='Fixed-rate logging frequency in Hz'
    )

    output_dir_arg = DeclareLaunchArgument(
        'output_dir',
        default_value=default_output_dir,
        description='Directory to write CSV logs'
    )

    micro_ros_agent_node = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        output='screen',
        arguments=['serial', '--dev', '/dev/esp', '--baudrate', '921600']
    )

    syringe_ui_node = Node(
        package='syringe_ui',
        executable='syringe_ui',
        name='syringe_ui',
        output='screen',
        parameters=[{
            'log_rate_hz': LaunchConfiguration('log_rate_hz'),
            'output_dir': LaunchConfiguration('output_dir'),
        }]
    )

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
        log_rate_arg,
        output_dir_arg,
        micro_ros_agent_node,
        syringe_ui_node,
        delta_ph_server_node,
        derivative_ph_volume_server_node,
        syringe_server_node,
    ])