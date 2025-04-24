import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
import time

class AnomalyResponder(Node):
    def __init__(self):
        super().__init__('anomaly_responder')
        self.subscription = self.create_subscription(Bool, '/anomaly_detected', self.callback, 10)
        self.publisher = self.create_publisher(Twist, '/navigation/cmd_vel', 10)
        self.timer = self.create_timer(2.0, self.forward_motion)
        self.anomaly_detected = False

    def forward_motion(self):
        if not self.anomaly_detected:
            twist = Twist()
            twist.linear.x = 0.1  
            self.publisher.publish(twist)

    def callback(self, msg):
        if msg.data:  # If anomaly detected
            self.anomaly_detected = True
            self.get_logger().warn("Obstacle/Crowd detected — rerouting...")

            stop_msg = Twist()
            self.publisher.publish(stop_msg)
            time.sleep(1.0)

            twist = Twist()
            twist.angular.z = 0.3
            self.publisher.publish(twist)
            time.sleep(1.2)

            self.anomaly_detected = False

def main():
    rclpy.init()
    node = AnomalyResponder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
