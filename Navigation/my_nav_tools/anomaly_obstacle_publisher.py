#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
from geometry_msgs.msg import Pose

class ObstaclePublisher(Node):
    def __init__(self):
        super().__init__('obstacle_publisher')
        self.publisher_ = self.create_publisher(OccupancyGrid, '/crowd_zones', 10)
        self.timer = self.create_timer(1.0, self.publish_obstacle_grid)

    def publish_obstacle_grid(self):
        grid = OccupancyGrid()
        grid.header = Header()
        grid.header.frame_id = 'map'
        grid.info.resolution = 0.2  # 20cm per cell
        grid.info.width = 50
        grid.info.height = 50
        grid.info.origin.position.x = 3.0
        grid.info.origin.position.y = 2.0

        # Fill in grid with dummy data: a 10x10 block in the center
        data = []
        for j in range(50):
            for i in range(50):
                if 20 <= i < 30 and 20 <= j < 30:
                    data.append(100)  # occupied
                else:
                    data.append(0)    # free
        grid.data = data

        self.publisher_.publish(grid)
        self.get_logger().info('Published obstacle grid to /crowd_zones')

def main(args=None):
    rclpy.init(args=args)
    node = ObstaclePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
