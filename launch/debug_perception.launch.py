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
    default_bt_xml_path = os.path.join(bt_pkg_dir, 'behavior_tree', 'bt_mission.xml')

    # Launch Arguments
    bt_xml_arg = DeclareLaunchArgument(
        'bt_xml_path',
        default_value=default_bt_xml_path,
        description='Path to the Behavior Tree XML file'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulated clock if true'
    )

    # Cone Detector Pipeline
    
    # 1. Scan to PointCloud
    scan_to_pcl_node = Node(
        package='cone_detector',
        executable='scan_to_pointcloud',
        name='scan_to_pointcloud',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        remappings=[
            ('/scan_filtered', '/scan')
        ]
    )

    # 2. Cone Clustering
    cone_cluster_node = Node(
        package='cone_detector',
        executable='cone_cluster_node',
        name='cone_cluster_node',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # 3. Cone Color Detector
    cone_color_detector_node = Node(
        package='cone_detector',
        executable='cone_color_detector_node',
        name='cone_color_detector_node',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        remappings=[
            ('/image_raw', '/camera/color/image_raw'),
            ('/camera_info', '/camera/color/camera_info')
        ]
    )

    # 4. Cone Fusion
    cone_fusion_node = Node(
        package='cone_detector',
        executable='cone_fusion_node',
        name='cone_fusion_node',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        remappings=[
            ('/camera_info', '/camera/color/camera_info')
        ]
    )

    # 5. Cone Area
    cone_area_node = Node(
        package='cone_detector',
        executable='cone_area_node',
        name='cone_area_node',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # Coverage Planner Node (Zigzag Generator)
    zigzag_node = Node(
        package='coverage_planner',
        executable='zigzag_generator',
        name='zigzag_generator',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    return LaunchDescription([
        bt_xml_arg,
        use_sim_time_arg,
        scan_to_pcl_node,
        cone_cluster_node,
        cone_color_detector_node,
        cone_fusion_node,
        cone_area_node,
        zigzag_node
    ])
