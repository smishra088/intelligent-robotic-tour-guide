import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
from geometry_msgs.msg import Twist
import cv2
import time

class YoloV8Node(Node):
    def __init__(self):
        super().__init__('yolov8_node')
        self.subscription = self.create_subscription(
            Image,
            '/head_front_camera/rgb/image_raw',
            self.listener_callback,
            10)
        self.bridge = CvBridge()
        self.model = YOLO('yolov8n.pt')
        self.cmd_vel_pub = self.create_publisher(Twist, '/robot/cmd_vel', 10)
        self.detected = False

    def listener_callback(self, msg):
        if self.detected:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            results = self.model(frame)
            annotated = results[0].plot()
            cv2.imshow("YOLOv8 Detection", annotated)
            cv2.waitKey(1)

            person_count = 0
            obstacle_detected = False

            for box in results[0].boxes:
                cls = int(box.cls[0].item())
                label = self.model.names.get(cls, "unknown")

                if label == 'person':
                    person_count += 1
                elif label in ['chair', 'bench', 'tv', 'bottle']:
                    obstacle_detected = True

            if person_count >= 1 or obstacle_detected:
                self.get_logger().warn(f"Crowd Detected: {person_count} people")
                self.detected = True

                twist = Twist()
                twist.angular.z = 0.5
                twist.linear.x = 0.1
                self.cmd_vel_pub.publish(twist)

                time.sleep(5)

                stop = Twist()
                self.cmd_vel_pub.publish(stop)

        except Exception as e:
            self.get_logger().error(f"YOLOv8 Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = YoloV8Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
