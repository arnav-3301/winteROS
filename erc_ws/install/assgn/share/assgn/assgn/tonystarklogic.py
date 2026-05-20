#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import time

class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_exploration')

        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber for LiDAR scan data
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)

        # Initialize control variables
        self.safe_distance = 0.7  # Fill blank 1: Minimum safe distance from obstacles
        self.is_turning = False   # Direction lock state
        self.twist = Twist()      # Velocity command

        self.get_logger().info("Obstacle Avoidance Node Initialized - Anti-Jitter Version")

    def lidar_callback(self, scan_data):
        # Get the minimum distance readings from LiDAR data
        num_readings = len(scan_data.ranges)
        left_side = min(scan_data.ranges[4*num_readings//5:]) 
        right_side = min(scan_data.ranges[0:num_readings//5]) 
        front = min(scan_data.ranges[2*num_readings//5:3*num_readings//5])

        if front < self.safe_distance or self.is_turning:
            # If front is blocked, check the left and right
            if not self.is_turning:
                self.is_turning = True # Lock the state
                if left_side > right_side:
                    self.get_logger().info("Obstacle in front. Turning left.")
                    self.twist.angular.z = 0.3  # Fill blank 2: Angular velocity for left turn
                else:
                    self.get_logger().info("Obstacle in front. Turning right.")
                    self.twist.angular.z = -0.3 # Fill blank 3: Angular velocity for right turn
            
            self.twist.linear.x = 0.0  # Stop moving forward while turning
            
            # Check if path is finally clear to stop turning
            if front > 1.5: 
                self.is_turning = False
        
        elif front < 0.4 and left_side < 0.4 and right_side < 0.4:
            # If all directions are blocked, move backward slightly
            self.get_logger().info("All directions blocked. Moving backward!")
            self.twist.linear.x = -0. # Fill blank 4: Linear velocity for backward motion
            self.twist.angular.z = 0.0
        
        else:
            # Path is clear, move forward
            self.is_turning = False
            self.twist.linear.x = 0.3  # Fill blank 5: Linear velocity for forward motion
            self.twist.angular.z = 0.0

        # Publish the velocity command
        self.cmd_vel_pub.publish(self.twist)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()  # Fill blank 6: Instance of the ObstacleAvoidanceNode class
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()