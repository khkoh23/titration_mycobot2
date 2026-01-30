from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    tree_path = PathJoinSubstitution ( [get_package_share_directory('titration_robot_syringe'), 'tree', 'syringe_equivalence.xml'])
    
    bt_node = Node(
        package='titration_robot_syringe',
        executable='syringe_equivalence_executor',
        output='screen',
        parameters=[{'tree_xml_file': tree_path}],
    )
    
    return LaunchDescription([bt_node])