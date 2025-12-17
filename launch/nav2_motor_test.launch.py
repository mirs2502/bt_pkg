import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Paths
    bt_pkg_dir = get_package_share_directory('bt_pkg')
    mirs_pkg_dir = get_package_share_directory('mirs')
    
    # Configuration Files
    default_bt_xml_path = os.path.join(bt_pkg_dir, 'behavior_tree', 'nav2_motor_test.xml')
    nav2_params_path = os.path.join(mirs_pkg_dir, 'config', 'nav2_params.yaml')

    # Launch Arguments
    bt_xml_arg = DeclareLaunchArgument(
        'bt_xml_path',
        default_value=default_bt_xml_path,
        description='Path to the Behavior Tree XML file'
    )

    # Nav2 Behavior Server
    # Provides Spin, Wait, BackUp, etc.
    behavior_server_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params_path],
        # Remap if necessary, but usually defaults are fine if using standard topics
        # remappings=[('cmd_vel', 'cmd_vel')] 
    )

    # Nav2 Lifecycle Manager
    # Required to transition behavior_server to Active state
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_behaviors',
        output='screen',
        parameters=[{
            'use_sim_time': False, # Set to True if running in simulation
            'autostart': True,
            'node_names': ['behavior_server']
        }]
    )

    # BT Executor Node (Our custom node)
    # Sends goals to behavior_server
    bt_executor_node = Node(
        package='bt_pkg',
        executable='bt_executor',
        name='bt_nav2_motor_test',
        output='screen',
        parameters=[{
            'bt_xml_path': LaunchConfiguration('bt_xml_path')
        }]
    )

    return LaunchDescription([
        bt_xml_arg,
        behavior_server_node,
        lifecycle_manager_node,
        bt_executor_node
    ])
