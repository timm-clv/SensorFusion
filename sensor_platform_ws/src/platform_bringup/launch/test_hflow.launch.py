
#===========================================
#==============CALLED BY NOBODY===========
#===========================================


# OLD file for test the H flow
# Don't needed and can be deleted if need



import os
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node

def generate_launch_description():
    
    set_domain_id = SetEnvironmentVariable('ROS_DOMAIN_ID', '3')

    uavcan_node = Node(
        package='uavcan_bridge',
        executable='dronecan_bridge_node', 
        name='hflow_can_bridge',
        parameters=[{
            'can_interface': 'slcan0',
            'node_id': 127
        }],
        output='screen'
    )

    return LaunchDescription([
        uavcan_node
    ])
