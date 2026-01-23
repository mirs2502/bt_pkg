#!/usr/bin/env python3
"""
2つのコーン間を往復するミッション用のLaunchファイル

起動するノード:
- コーン検出パイプライン（scan_to_pointcloud, cone_cluster, cone_color_detector, cone_fusion, cone_area）
- two_cone_reciprocate（2点間往復パス生成）
- bt_executor（Behavior Tree実行）
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # パッケージディレクトリ
    bt_pkg_dir = get_package_share_directory('bt_pkg')

    # デフォルトBT XMLパス
    default_bt_xml_path = os.path.join(bt_pkg_dir, 'behavior_tree', 'bt_two_cone.xml')

    # Launch引数
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

    # --- コーン検出パイプライン ---

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
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'max_area': 200000
        }],
        remappings=[
            ('/image_raw', '/camera/color/image_raw'),
            ('/camera_info', '/camera/color/camera_info'),
            ('/color_mask', '/color_mask')
        ]
    )

    # 4. Cone Fusion
    cone_fusion_node = Node(
        package='cone_detector',
        executable='cone_fusion_node',
        name='cone_fusion_node',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'use_lidar_only': LaunchConfiguration('use_lidar_only')
        }]
    )

    # 5. Cone Area（蓄積）
    cone_area_node = Node(
        package='cone_detector',
        executable='cone_area_node',
        name='cone_area_node',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'target_frame': 'odom'
        }]
    )

    # --- パスジェネレータ ---

    # 2コーン往復パスジェネレータ
    two_cone_reciprocate_node = Node(
        package='coverage_planner',
        executable='two_cone_reciprocate',
        name='two_cone_reciprocate',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'stop_distance': 0.3,
            'path_resolution': 0.3,
            'num_cycles': 3  # 3往復
        }]
    )

    # --- Behavior Tree Executor ---

    bt_executor_node = Node(
        package='bt_pkg',
        executable='bt_executor',
        name='bt_main',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'bt_xml_filename': LaunchConfiguration('bt_xml_path')
        }]
    )

    # BT Executorを遅延起動（他のノードが準備できるまで待つ）
    delayed_bt_executor = TimerAction(
        period=3.0,
        actions=[bt_executor_node]
    )

    return LaunchDescription([
        bt_xml_arg,
        use_sim_time_arg,
        use_lidar_only_arg,

        # コーン検出パイプライン
        scan_to_pcl_node,
        cone_cluster_node,
        cone_color_detector_node,
        cone_fusion_node,
        cone_area_node,

        # パスジェネレータ
        two_cone_reciprocate_node,

        # Behavior Tree Executor（遅延起動）
        delayed_bt_executor
    ])
