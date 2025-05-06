import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

class CameraTestNode(Node):
    def __init__(self):
        super().__init__('camera_test_node')
        self.subscription = self.create_subscription(
            Image,
            '/head_front_camera/rgb/image_raw',
            self.image_callback,
            10
        )
        self.get_logger().info("CameraTestNode started, waiting for images...")

    def image_callback(self, msg):
        self.get_logger().info("✅ Received an image!")

def main(args=None):
    rclpy.init(args=args)
    node = CameraTestNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
