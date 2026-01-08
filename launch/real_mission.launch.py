import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
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

    use_lidar_only_arg = DeclareLaunchArgument(
        'use_lidar_only',
        default_value='false',
        description='If true, skip camera fusion and use lidar only'
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
            ('/scan_filtered', '/scan') # Assuming real robot publishes to /scan
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

    # 3. Cone Color Detector (Real Robot only)
    # Assuming camera publishes to /camera/color/image_raw and /camera/color/camera_info
    # Adjust remappings if necessary based on real hardware setup
    cone_color_detector_node = Node(
        package='cone_detector',
        executable='cone_color_detector_node',
        name='cone_color_detector_node',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'max_area': 200000  # コーンが近くて大きく映る場合に対応
        }],
        remappings=[
            ('/image_raw', '/camera/color/image_raw'),
            ('/camera_info', '/camera/color/camera_info'),
            ('/color_mask', '/color_mask') # Explicitly remap if needed, though default is fine. Added for clarity.
        ]
    )

    # 4. Cone Fusion (Real Robot only)
    cone_fusion_node = Node(
        package='cone_detector',
        executable='cone_fusion_node',
        name='cone_fusion_node',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'use_lidar_only': LaunchConfiguration('use_lidar_only')
        }],
        remappings=[
            ('/camera_info', '/camera/color/camera_info')
        ]
    )

    # 5. Cone Area (Polygon Generation)
    cone_area_node = Node(
        package='cone_detector',
        executable='cone_area_node',
        name='cone_area_node',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'threshold_dist': 0.8  # 重複判定の距離閾値 (m)
        }]
        # No remapping needed for /confirmed_cones as fusion node publishes to it
    )

    # Behavior Tree Node
    bt_node = Node(
        package='bt_pkg',
        executable='bt_executor',
        name='bt_main',
        output='screen',
        parameters=[{
            'bt_xml_path': LaunchConfiguration('bt_xml_path'),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    # Coverage Planner Node (Zigzag Generator)
    zigzag_node = Node(
        package='coverage_planner',
        executable='zigzag_generator',
        name='zigzag_generator',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # Delay BT Node to wait for Nav2 to be ready
    delayed_bt_node = TimerAction(
        period=5.0,
        actions=[bt_node]
    )

    return LaunchDescription([
        bt_xml_arg,
        use_sim_time_arg,
        use_lidar_only_arg,
        scan_to_pcl_node,
        cone_cluster_node,
        cone_color_detector_node,
        cone_fusion_node,
        cone_area_node,
        zigzag_node,
        delayed_bt_node
    ])
