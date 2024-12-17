import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Get the share directory of the feature_tracker package
    # feature_tracker_share = get_package_share_directory('feature_tracker')
    # feature_tracker_src = get_pa

    # Construct the paths to your configuration files
    # config_path = os.path.join(feature_tracker_share, '..', 'config', 'euroc', 'euroc_config.yaml')
    config_path = '/root/aerostack2_ws/src/VINS-Mono/config/euroc/euroc_config.yaml'
    # vins_path = os.path.join(feature_tracker_share, '..', 'config', '..')
    vins_path = '/root/aerostack2_ws/src/VINS-Mono/'

    # Define nodes with parameters
    feature_tracker_node = Node(
        package='feature_tracker',
        executable='feature_tracker',
        name='feature_tracker',
        namespace='feature_tracker',
        output='log',
        parameters=[{
            'config_file': config_path,
            'vins_folder': vins_path
        }]
    )

    vins_estimator_node = Node(
        package='vins_estimator',
        executable='vins_estimator',
        name='vins_estimator',
        namespace='vins_estimator',
        output='screen',
        parameters=[{
            'config_file': config_path,
            'vins_folder': vins_path
        }]
    )

    pose_graph_node = Node(
        package='pose_graph',
        executable='pose_graph',
        name='pose_graph',
        namespace='pose_graph',
        output='screen',
        parameters=[{
            'config_file': config_path,
            'vins_folder': vins_path,
            'visualization_shift_x': 0,
            'visualization_shift_y': 0,
            'skip_cnt': 0,
            'skip_dis': 0.0
        }]
    )

    return LaunchDescription([
        feature_tracker_node,
        vins_estimator_node,
        pose_graph_node
    ])

