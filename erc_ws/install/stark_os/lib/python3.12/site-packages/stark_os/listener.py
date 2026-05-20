#!/usr/bin/env python
import rclpy
from std_msgs.msg import String

def listener_callback(msg):
    print('I heard: "%s"' % msg.data)

def main(args=None):

    rclpy.init(args=args)

    node = rclpy.create_node('hud_sub')

    subscription1 = node.create_subscription(String, 'reactor_status', listener_callback, 10)
    subscription2 = node.create_subscription(String, 'system_diag', listener_callback, 10)

    subscription1
    subscription2

    try:

        rclpy.spin(node)

    finally:

        node.destroy_node()

        rclpy.shutdown()

if __name__ == '__main__':
    main()