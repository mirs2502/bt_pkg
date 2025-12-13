import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Paths
    bt_pkg_dir = get_package_share_directory('bt_pkg')
    
    # Default BT XML path
    default_bt_xml_path = os.path.join(bt_pkg_dir, 'behavior_tree', 'motor_test_topic.xml')

    # Launch Arguments
    bt_xml_arg = DeclareLaunchArgument(
        'bt_xml_path',
        default_value=default_bt_xml_path,
        description='Path to the Behavior Tree XML file'
    )

    # Behavior Tree Node
    bt_node = Node(
        package='bt_pkg',
        executable='bt_executor',
        name='bt_motor_test',
        output='screen',
        parameters=[{
            'bt_xml_path': LaunchConfiguration('bt_xml_path')
        }]
    )

    return LaunchDescription([
        bt_xml_arg,
        bt_node
    ])
