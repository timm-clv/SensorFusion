

#===========================================
#== CALLED BY system.launch.py===========
#===========================================

# Almost all is done in : config -> rs_custom -> estimator_config.yaml
# decompression of the image topic of the OliveVision camera is made here
#But if you want to pass with this camera you have to change the topic and parameter configuration in estimator_config.yaml and cam and embedded IMU
# Like it's a monocular camera, it's possible that doesn't work



import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_bringup = get_package_share_directory('platform_bringup')
    ov_config = os.path.join(pkg_bringup, 'config', 'rs_custom', 'estimator_config.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo/Rosbag) clock'
    )

    return LaunchDescription([
        declare_use_sim_time,


        Node(
            package='platform_bringup',
            executable='decompressor.py',
            name='decompressor_node',
            remappings=[
                # Rename
                ('/olivecam/image/compressed', '/olive/camera/olivecam/image/compressed')
            ],
            # decompression and publish on /olive/camera/image_raw (waited by OpenVINS)
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        Node(
            package='ov_msckf',
            executable='run_subscribe_msckf',
            name='ov_msckf',
            namespace='ov_msckf',
            arguments=[ov_config], 
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),

    ])
