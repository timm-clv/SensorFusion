#===========================================
#== CALLED BY system.launch.py===========
#===========================================

# file for launch the EKF cleanly


import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_bringup = get_package_share_directory('platform_bringup')
    ekf_config_path = os.path.join(pkg_bringup, 'config', 'ekf.yaml')

    # Option to synchronize the clock with a Rosbag if necessary
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo/Rosbag) clock'
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_config_path, 
            {'use_sim_time': use_sim_time} 
        ],
        remappings=[
            ('odometry/filtered', 'odometry/filtered_ekf'),
            ('accel/filtered', 'accel/filtered_ekf')
        ]
    )

    return LaunchDescription([
        declare_use_sim_time,
        ekf_node
    ])
