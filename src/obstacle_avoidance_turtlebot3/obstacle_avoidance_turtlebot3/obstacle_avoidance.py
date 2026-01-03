#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')

        # Subscriber to LIDAR
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # Publisher to cmd_vel
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.cmd = Twist()
        self.safe_distance = 0.5  # meters

    def scan_callback(self, msg):
        # Divide LIDAR into 3 regions
        front = min(min(msg.ranges[0:20]), min(msg.ranges[340:360]))
        left  = min(msg.ranges[20:100])
        right = min(msg.ranges[260:340])

        self.get_logger().info(
            f"Front: {front:.2f}, Left: {left:.2f}, Right: {right:.2f}"
        )

        # MAIN LOGIC
        if front < self.safe_distance:
            self.get_logger().info("Obstacle Ahead → Turning Right")
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = -0.6

        elif left < self.safe_distance:
            self.get_logger().info("Obstacle Left → Turning Right")
            self.cmd.linear.x = 0.15
            self.cmd.angular.z = -0.4

        elif right < self.safe_distance:
            self.get_logger().info("Obstacle Right → Turning Left")
            self.cmd.linear.x = 0.15
            self.cmd.angular.z = 0.4

        else:
            self.get_logger().info("Path Clear → Moving Forward")
            self.cmd.linear.x = 0.22
            self.cmd.angular.z = 0.0
        
        # Publish
        self.cmd_pub.publish(self.cmd)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidance()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

