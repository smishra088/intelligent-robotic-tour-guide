import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraDebugNode(Node):
    def __init__(self):
        super().__init__('camera_debug_node')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/head_front_camera/rgb/image_raw',
            self.image_callback,
            10
        )
        self.get_logger().info("📡 CameraDebugNode active")

    def image_callback(self, msg):
        self.get_logger().info("✅ Got an image!")
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv2.imwrite("/tmp/debug_image.jpg", frame)
        self.get_logger().info("💾 Saved frame to /tmp/debug_image.jpg")

def main(args=None):
    rclpy.init(args=args)
    node = CameraDebugNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

