#!/usr/bin/env python3


#====================================
#== CALLED BY SYSTEM.LAUNCH===========
#==================================



import rclpy
from rclpy.node import Node
import dronecan

class DronecanAllocator(Node):
    def __init__(self):
        super().__init__('dronecan_allocator')
        try:
            # Jetson take the ID 127 (in the can)
            self.node = dronecan.make_node('slcan0', node_id=127, bitrate=1000000)
            # critic : initialised the "NodeMonitor"
            self.node_monitor = dronecan.app.node_monitor.NodeMonitor(self.node)
            # launch
            self.allocator = dronecan.app.dynamic_node_id.CentralizedServer(self.node, self.node_monitor)
            
            self.timer = self.create_timer(0.01, lambda: self.node.spin(timeout=0.005))
            self.get_logger().info("DroneCAN allocation server active on slcan0")
        except Exception as e:
            self.get_logger().error(f"Errorr CAN: {e}")

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(DronecanAllocator())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
