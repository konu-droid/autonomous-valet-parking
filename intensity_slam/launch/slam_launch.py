import os
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    # static transform between map and odom frame
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )

    # My simple SLAM Node
    slam_node = Node(
        package='intensity_slam',
        executable='simple_slam',
        name='simple_slam_node',
        output='screen',
        parameters=[{'use_sim_time': True}],  # Set simulation time here
    )

    # Laser Filter Node
    filter_node = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='laser_filter_node',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                get_package_share_directory("intensity_slam"),
                "config", "filter_chain.yaml",
            ])],
        remappings=[
            ('/scan', '/front_2d_lidar/scan'),
            ('/scan_filtered', '/scan'),
        ]
    )

    # RViz Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([
                get_package_share_directory("intensity_slam"),
                "config", "mapping.rviz",
            ])],
    )
    
    return LaunchDescription([
        static_tf,
        # slam_node,
        filter_node,
        # rviz_node,
    ])