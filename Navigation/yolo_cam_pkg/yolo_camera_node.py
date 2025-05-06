import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

class YOLOCameraNode(Node):
    def __init__(self):
        super().__init__('yolo_camera_node')
        self.get_logger().info("🚀 YOLO Camera Node started")
        self.subscription = self.create_subscription(
            Image,
            '/head_front_camera/rgb/image_raw',
            self.image_callback,
            10
        )
        self.bridge = CvBridge()
        self.model = YOLO('yolov8n.pt')  # or yolov8s.pt, etc.

    def image_callback(self, msg):
        self.get_logger().info("📷 Received image for detection")

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = self.model(frame)[0]
        annotated = results.plot()

        cv2.imshow("YOLOv8 Detection", annotated)
        cv2.waitKey(1)
        cv2.imwrite("/tmp/yolo_debug_output.jpg", annotated)

def main(args=None):
    rclpy.init(args=args)
    node = YOLOCameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

