
# Give a display of the platform 
# Great for visual debugging and publish transforms



import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_share = get_package_share_directory('platform_description')
    
    # Process the XACRO file
    xacro_file = os.path.join(pkg_share, 'urdf', 'sensor_platform.urdf.xacro')
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    return LaunchDescription([
        # Node 1: Robot State Publisher (publishes static/dynamic TFs)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_raw}]
        ),        

        # Node 2: RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        )
    ])
