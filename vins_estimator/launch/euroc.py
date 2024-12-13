#!/bin/env/python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
    config_path_arg = DeclareLaunchArgument(
        'config_path', 
        default_value=['$(find feature_tracker)/../config/euroc/euroc_config.yaml'], 
        description='Path to the config file'
    )
    
    vins_path_arg = DeclareLaunchArgument(
        'vins_path', 
        default_value=['$(find feature_tracker)/../config/../'],
        description='Path to the VINS folder'
    )
    # Define nodes
    feature_tracker_node = Node(
        package='feature_tracker',
        executable='feature_tracker',
        name='feature_tracker',
        output='log',
        parameters=[{
            'config_file': LaunchConfiguration('config_path'),
            'vins_folder': LaunchConfiguration('vins_path')
        }]
    )

    vins_estimator_node = Node(
        package='vins_estimator',
        executable='vins_estimator',
        name='vins_estimator',
        output='screen',
        parameters=[{
            'config_file': LaunchConfiguration('config_path'),
            'vins_folder': LaunchConfiguration('vins_path')
        }]
    )

    pose_graph_node = Node(
        package='pose_graph',
        executable='pose_graph',
        name='pose_graph',
        output='screen',
        parameters=[{
            'config_file': LaunchConfiguration('config_path'),
            'visualization_shift_x': 0,
            'visualization_shift_y': 0,
            'skip_cnt': 0,
            'skip_dis': 0.0
        }]
    )

    return LaunchDescription([
        config_path_arg,
        vins_path_arg,
        feature_tracker_node,
        vins_estimator_node,
        pose_graph_node
    ])
