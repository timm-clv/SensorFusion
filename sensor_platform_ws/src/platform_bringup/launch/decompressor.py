#!/usr/bin/env python3


#===========================================
#== CALLED BY openvins.launch.py===========
#===========================================

#========== Needed if camera OlixVision ===========
#==== Don't need this if D435i================



import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np
import cv2

class DecompressorNode(Node):
    def __init__(self):
        super().__init__('decompressor_node')
        self.bridge = CvBridge()
        
        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.sub = self.create_subscription(
            CompressedImage,
            '/olive/camera/olivecam/image/compressed',
            self.image_callback,
            qos_best_effort
        )
        
        self.pub = self.create_publisher(Image, '/olive/camera/image_raw', 10)
        self.get_logger().info('Active heavy-duty decompressor (Numpy/OpenCV)...')

    def image_callback(self, msg):
        try:
            # conversion in octets array
            np_arr = np.frombuffer(msg.data, np.uint8)
            
            # Open CV decompressor (best)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            # Verification ~"image is full"
            if cv_image is None:
                self.get_logger().warn('Corrupted frame ignored (Frame Drop).')
                return

            # Conversion to a standard ROS 2 message
            raw_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            raw_msg.header = msg.header 
            
            self.pub.publish(raw_msg)
            
        except Exception as e:
            self.get_logger().error(f'Fatal decompression error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = DecompressorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
