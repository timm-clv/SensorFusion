


#====================================
#========= CALLED BY NOBODY===========
#==================================


#==========test for later (with isaac ros)==========





import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    urdf_path = os.path.join(
        get_package_share_directory('platform_description'),
        'urdf',
        'sensor_platform.urdf.xacro'
    )
    robot_description_config = xacro.process_file(urdf_path).toxml()

    # 2. Robot State Publisher node
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_config,
            'use_sim_time': True # if rosbag
        }]
    )

    # 3. node Isaac ROS VSLAM 
    vslam_node = ComposableNode(
        name='visual_slam_node',
        package='isaac_ros_visual_slam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        parameters=[{
            'use_sim_time': True, 
            'enable_image_denoising': False,
            'rectified_images': True,
            'enable_imu_fusion': True,
            # VÉRIFICATION CRITIQUE : Ces frames doivent exister dans ton URDF
            'base_frame': 'base_link',
            'odom_frame': 'odom',
            'map_frame': 'map',
            'camera_optical_frame': 'olivecam_optical_frame', # depending URDF
            'imu_frame': 'imu_link'                           # depending URDF
        }],
        remappings=[
            ('visual_slam/image_0', '/olive/camera/image_raw'),
            ('visual_slam/camera_info_0', '/olivecam/camera_info_synced'),
            ('visual_slam/imu', '/imu/fused')
        ]
    )

    vslam_container = ComposableNodeContainer(
        name='visual_slam_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[vslam_node],
        output='screen'
    )

    return LaunchDescription([
        rsp_node,
        vslam_container
    ])
