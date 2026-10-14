"""Launch the MTracker driver and its fixed sensor transforms."""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    config = PathJoinSubstitution([
        FindPackageShare('mtracker'),
        'config',
        'mtracker.yaml',
    ])

    return LaunchDescription([
        Node(
            package='mtracker',
            executable='mtracker_drv',
            name='mtracker',
            parameters=[config],
            output='screen',
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_test',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_base_link_camera',
            arguments=['1', '-1', '0', '0.7', '0', '0',
                       'map', 'base_link_camera'],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_laser',
            arguments=['0.01', '0', '0', '0', '0', '0',
                       'base_link', 'laser'],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_base_footprint',
            arguments=['0', '0', '0', '0', '0', '0',
                       'base_link', 'base_footprint'],
        ),
    ])
