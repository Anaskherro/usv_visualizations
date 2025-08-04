import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('robot_localization_pkg'),
        'config',
        'ekf_config.yaml'
    )

    return LaunchDescription([
        launch_ros.actions.Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[config_file]
        )
    ])

