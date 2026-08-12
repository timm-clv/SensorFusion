import os
from datetime import datetime
from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    # Bag name
    bag_name = datetime.now().strftime('UKF1')
    
    topics_to_record = [
        # --- Geometry and ground truth ---
        '/tf',
        '/tf_static',
        '/robot_description',
        '/RigidBody_004/pose', # to adapt depending of OptiTrack
        
        # --- Sensors Olive and Holybro ---
        #'/olivecam/image/compressed',
        #'/imu/fused',
        #'/olive/olixSense/x1/oliveimu/imu',
        #'/olivecam/filtered_ahrs_synced',
        '/optical_flow/velocity',
        '/optical_flow/range',
        
        # --- Sensor RealSense : Stereo/VIO ---
        #'/d435/d435_node/imu',
        #'/d435/d435_node/infra1/image_rect_raw',
        #'/d435/d435_node/infra2/image_rect_raw',
        #'/d435/d435_node/infra1/camera_info',
        #'/d435/d435_node/infra2/camera_info',
        
        # --- Pose estimation ---
        '/odometry/filtered_ukf',
        '/odometry/filtered_ekf',
        '/ov_msckf/odomimu'
    ]
    

    record_command = ['ros2', 'bag', 'record',
    '--max-bag-size', '2147483648',   # size of files
    '--compression-mode', 'message',  
    '--compression-format', 'zstd',
    '-o', bag_name] + topics_to_record

    return LaunchDescription([
        ExecuteProcess(
            cmd=record_command,
            output='screen'
        )
    ])
