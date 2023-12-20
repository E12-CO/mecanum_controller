# Launch file for menanum controller for ABU Robocon 2024
import os

import launch
import launch_ros.actions
import launch_ros.descriptions

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_name = 'mecanum_controller'

    mecanum_param_dir = launch.substitutions.LaunchConfiguration(
        'mecanum_param_dir',
        default=os.path.join(
            get_package_share_directory(pkg_name),
            'params',
            'mecanum.yaml'))
    
    mecanum_controller_instant = launch_ros.actions.Node(
        package='mecanum_controller',
        executable='mecanum_controller',
        output='screen',
        parameters=[mecanum_param_dir]
    )
    
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'mecanum_param_dir',
            default_value=mecanum_param_dir,
            description='Full path to main parameter file to load'),
        mecanum_controller_instant,
    ])