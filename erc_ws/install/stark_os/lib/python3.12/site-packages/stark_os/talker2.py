#!/usr/bin/env python
import rclpy
from std_msgs.msg import String

def timer_callback(timer, i):
    msg = String()
    msg.data = 'Suit integrity at 85%. All systems responsive.'

    publisher.publish(msg)

    print('Publishing: "%s"' % msg.data)

def main(args=None):

    rclpy.init(args=args)

    node = rclpy.create_node('sys_pub')

    global publisher
    publisher = node.create_publisher(String, 'system_diag', 10)

    timer_period = 0.5

    i = 0

    timer = node.create_timer(timer_period, lambda: timer_callback(timer, i))

    i += 1

    try:

        rclpy.spin(node)
    finally:

        node.destroy_node()

        rclpy.shutdown()

if __name__ == '__main__':
    main()