import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Paths
    bt_pkg_dir = get_package_share_directory('bt_pkg')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # World file
    world_file = os.path.join(bt_pkg_dir, 'worlds', 'cone_world.world')

    # Nav2 Simulation Launch
    nav2_sim_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'tb3_simulation_launch.py')
        ),
        launch_arguments={
            'headless': 'False', # Show Gazebo GUI
            'world': world_file,
            'x_pose': '0.0',
            'y_pose': '0.0',
            'z_pose': '0.01'
        }.items()
    )

    # Cone Detector Pipeline
    
    # 1. Scan to PointCloud
    scan_to_pcl_node = Node(
        package='cone_detector_package',
        executable='scan_to_pointcloud',
        name='scan_to_pointcloud',
        output='screen'
    )

    # 2. Cone Clustering
    cone_cluster_node = Node(
        package='cone_detector_package',
        executable='cone_cluster_node',
        name='cone_cluster_node',
        output='screen'
    )

    # 3. Cone Area (Polygon Generation)
    # Remap /confirmed_cones to /cone_centers to bypass fusion node (since we don't have camera in sim yet)
    cone_area_node = Node(
        package='cone_detector_package',
        executable='cone_area_node',
        name='cone_area_node',
        output='screen',
        remappings=[
            ('/confirmed_cones', '/cone_centers')
        ]
    )

    return LaunchDescription([
        nav2_sim_cmd,
        scan_to_pcl_node,
        cone_cluster_node,
        cone_area_node
    ])
