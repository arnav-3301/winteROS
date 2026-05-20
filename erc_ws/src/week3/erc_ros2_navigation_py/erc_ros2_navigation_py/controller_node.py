import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import time

class ExploreController(Node):
    def __init__(self):
        super().__init__('explore_controller')
        self.sub = self.create_subscription(Bool, '/object_detected', self.detected_callback, 10)
        self.pub = self.create_publisher(Bool, '/explore/resume', 10)
        
        self.current_state = None  # Track current state to avoid republishing
        self.last_publish_time = 0
        self.publish_cooldown = 1.0  # 1 second cooldown between state changes

    def detected_callback(self, msg):
        current_time = time.time()
        
        # Avoid rapid state changes
        if current_time - self.last_publish_time < self.publish_cooldown:
            return
        
        # Only publish if state actually changed
        if msg.data == self.current_state:
            return
        
        control_msg = Bool()
        
        if msg.data:
            # Object detected - STOP exploration
            self.get_logger().info("🔴 Object detected! Stopping exploration...")
            control_msg.data = False  # False = stop exploring
            self.pub.publish(control_msg)
            self.current_state = True
            
        else:
            # Object lost - RESUME exploration
            self.get_logger().info("🟢 Object lost! Resuming exploration...")
            control_msg.data = True  # True = resume exploring
            self.pub.publish(control_msg)
            self.current_state = False
        
        self.last_publish_time = current_time


def main(args=None):
    rclpy.init(args=args)
    node = ExploreController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()