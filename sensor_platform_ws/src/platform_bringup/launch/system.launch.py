
#===========================================
#======= BASE FILE OF THE SYSTEM ===========
#===========================================

#launch topics of the IMU, Hflow, camera, sensorfusion, TF or the sensor platform




import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction, SetEnvironmentVariable, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
import xacro

def generate_launch_description():

    SetEnvironmentVariable('ROS_DOMAIN_ID', '3')
       
    pkg_desc = get_package_share_directory('platform_description')
    pkg_bringup = get_package_share_directory('platform_bringup')
   
    # tests : 
    #pkg_networking = get_package_share_directory('olive_networking')
    #ekf_config = os.path.join(pkg_bringup, 'config', 'ekf.yaml')
    #inekf_config = os.path.join(pkg_bringup, 'config', 'inekf.yaml')
    #fusion_config = os.path.join(pkg_bringup, 'config', 'fusioncore.yaml')
    #realsense_config = os.path.join(pkg_bringup, 'config', 'realsense.yaml')

    

    # DroneCAN for Hflow
    allocator_script_path = os.path.join(pkg_bringup, 'launch', 'can_allocator.py')
    hflow_allocator = ExecuteProcess(
        cmd=['python3', allocator_script_path],
        output='screen'
    )

    # merging nodes into a single single-threaded container (zero-copy)
    sensor_processing_container = ComposableNodeContainer(
        name='sensor_processing_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt', # use of single-threaded (more light)
        composable_node_descriptions=[
            # Componant 1 : Material bridge DroneCAN
            ComposableNode(
                package='uavcan_bridge',
                plugin='uavcan_bridge::DronecanBridgeComponent',
                name='hflow_can_bridge',
                parameters=[{
                    'can_interface': 'slcan0',
                    'node_id': 127
                }],
                extra_arguments=[{'use_intra_process_comms': True}] # zero-copy active
            ),
            # Componant 2 : IMU
            ComposableNode(
                package='platform_processing',
                plugin='platform_processing::ImuSyncComponent',
                name='imu_sync_node',
                extra_arguments=[{'use_intra_process_comms': True}] # zero-copy active
            )
        ],
        output='screen'
    )
    
    # Staggered loading of the RealSense to avoid USB crashes
    load_realsense = LoadComposableNodes(
        target_container='sensor_processing_container',
        composable_node_descriptions=[
            ComposableNode(
                package='realsense2_camera',
                plugin='realsense2_camera::RealSenseNodeFactory',
                name='d435_node',
                namespace='d435',
                parameters=[{
                    'camera_name': 'd435',
                    'enable_gyro': True,
                    'enable_accel': True,
                    'unite_imu_method': 2,
                    'gyro_fps': 200,
                    'accel_fps': 250,
                    'depth_module.emitter_enabled': 0,
                    'enable_depth': False,
                    'enable_color': False,
                    'enable_infra1': True,
                    'enable_infra2': True,
                    'depth_module.profile': '848x480x30',
                    'enable_sync': True,
                    'initial_reset': True,
                    'serial_no': '108322073329',
                    'publish_tf': False,
                    'global_time_enabled': False  # Disables UVC XU requests that cause the Jetson's USB bus to crash
                }],
                extra_arguments=[{'use_intra_process_comms': True}]
            )
        ]
    )
    delayed_realsense = TimerAction(period=5.0, actions=[load_realsense])


    # Robot URDF Model (Single geometric source of truth – REP-105)
    xacro_file = os.path.join(pkg_desc, 'urdf', 'sensor_platform.urdf.xacro')
    robot_description_raw = xacro.process_file(xacro_file).toxml()
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw}]
    )      
    
    # VRPN Client (= ground truth given by the motion capture software Motive)
    vrpn_node = Node(
        package='vrpn_mocap',
        executable='client_node',
        name='vrpn_client',
        parameters=[{
            'server': '192.168.42.128', 
            'port': 3883,
            'update_freq': 100.0,     
            'frame_id': 'map',        
            'use_sim_time': False,
        }],
        output='screen'
    )
    
    
    
    # UKF launch
    ukf_launch = TimerAction(
        period=15.0, # period shift to be secure
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                   os.path.join(pkg_bringup, 'launch', 'ukf_fusion.launch.py')
                ),
                launch_arguments={'use_sim_time': 'false'}.items()
            )
        ]
    )
    
    
    # EKF launch
    ekf_launch = TimerAction(
        period=16.0, 
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                   os.path.join(pkg_bringup, 'launch', 'ekf_fusion.launch.py')
                ),
                launch_arguments={'use_sim_time': 'false'}.items()
            )
        ]
    )
    


    # OpenVins launch 
    delayed_openvins = TimerAction(
        period=7.0, # Allow 7 seconds to ensure the CAN bridge, ImuSync, and camera are already publishing
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_bringup, 'launch', 'openvins.launch.py')
                ),
                launch_arguments={'use_sim_time': 'false'}.items()
            )
        ]
    )
    
    
    
    
    return LaunchDescription([
        hflow_allocator,
        rsp_node,
        sensor_processing_container,
        delayed_realsense,
        vrpn_node,
        ukf_launch,
        ekf_launch,
        delayed_openvins
    ])
