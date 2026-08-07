#!/usr/bin/env python3


#===============================
#======CALLED BY NOBODY==========
#================================


# Was used to learn how to handle multiple IMU publication
# Not accurate in our actual setup
# Can be deleted





import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import random

class MockImuNode(Node):
    def __init__(self):
        super().__init__('mock_imu_node')
        
        self.pub_olive = self.create_publisher(Imu, '/olive/imu/data', 10)
        self.pub_xsens1 = self.create_publisher(Imu, '/xsens1/imu/data', 10)
        self.pub_xsens2 = self.create_publisher(Imu, '/xsens2/imu/data', 10)
        
        self.timer = self.create_timer(0.002, self.timer_callback)
        self.get_logger().info("Mock IMU Node démarré (1000 Hz).")

    def timer_callback(self):
        now = self.get_clock().now()
        
        self.publish_imu(self.pub_olive, now, 0.0)
        
        self.publish_imu(self.pub_xsens1, now, random.uniform(-0.0005, 0.0005))
        self.publish_imu(self.pub_xsens2, now, random.uniform(-0.0005, 0.0005))

    def publish_imu(self, publisher, base_time, jitter_sec):
        msg = Imu()
        
        jittered_time = base_time.nanoseconds + int(jitter_sec * 1e9)
        msg.header.stamp.sec = int(jittered_time // 1e9)
        msg.header.stamp.nanosec = int(jittered_time % 1e9)
        msg.header.frame_id = "mock_frame"
        
        msg.linear_acceleration.x = 9.81 + random.uniform(-0.1, 0.1)
        
        publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MockImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
