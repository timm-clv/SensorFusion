
#===========================================
#== CALLED BY system.launch.py===========
#===========================================

# file for launch the UKF cleanly



import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_bringup = get_package_share_directory('platform_bringup')
    ukf_config_path = os.path.join(pkg_bringup, 'config', 'ukf.yaml')

    # Argument to synchronize the clock with a rosbag (use_sim_time) if need
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo/Rosbag) clock'
    )


    ukf_node = Node(
        package='robot_localization',
        executable='ukf_node',
        name='ukf_filter_node',
        output='screen',
        parameters=[
            ukf_config_path, 
            {'use_sim_time': use_sim_time} 
        ],
        remappings=[ #added just for debugg due to an issue
            ('odometry/filtered', 'odometry/filtered_ukf'),
            ('accel/filtered', 'accel/filtered_ukf')
        ]
    )



    return LaunchDescription([
        declare_use_sim_time,
        ukf_node
    ])
