import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
import cv2
import numpy as np
import threading
from std_msgs.msg import Bool

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        
        # Create a subscriber with a queue size of 1 to only keep the last frame
        self.subscription = self.create_subscription(
            Image,
            'camera/image',
            self.image_callback,
            1  # Queue size of 1
        )

        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Initialize CvBridge
        self.bridge = CvBridge()
        
        # Variable to store the latest frame
        self.latest_frame = None
        self.frame_lock = threading.Lock()  # Lock to ensure thread safety
        
        # Flag to control the display loop
        self.running = True

        # Start a separate thread for spinning (to ensure image_callback keeps receiving new frames)
        self.spin_thread = threading.Thread(target=self.spin_thread_func)
        self.spin_thread.start()
        self.object_detected_pub = self.create_publisher(Bool, "/object_detected", 10)
        self.ball_detected = False
        self.stopped_near_ball = False        
        self.area_threshold = 0.25

    def spin_thread_func(self):
        """Separate thread function for rclpy spinning."""
        while rclpy.ok() and self.running:
            rclpy.spin_once(self, timeout_sec=0.05)

    def image_callback(self, msg):
        """Callback function to receive and store the latest frame."""
        # Convert ROS Image message to OpenCV format and store it
        with self.frame_lock:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def stop(self):
        """Stop the node and the spin thread."""
        self.running = False
        self.spin_thread.join()
    # Add small images to the top row of the main image
    def add_small_pictures(self, img, small_images, size=(160, 120)):

        x_base_offset = 40
        y_base_offset = 10

        x_offset = x_base_offset
        y_offset = y_base_offset

        for small in small_images:
            small = cv2.resize(small, size)
            if len(small.shape) == 2:
                small = np.dstack((small, small, small))

            img[y_offset: y_offset + size[1], x_offset: x_offset + size[0]] = small

            x_offset += size[0] + x_base_offset

        return img

    def display_image(self):
        """Main loop to process and display the latest frame."""
        # Create a single OpenCV window
        cv2.namedWindow("frame", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("frame", 800,600)

        while rclpy.ok():
            # Check if there is a new frame available
            if self.latest_frame is not None:

                # Process the current image
                mask, contour, crosshair = self.process_image(self.latest_frame)

                # Add processed images as small images on top of main image
                result = self.add_small_pictures(self.latest_frame, [mask, contour, crosshair])

                # Show the latest frame
                cv2.imshow("frame", result)
                self.latest_frame = None  # Clear the frame after displaying

            # Check for quit key
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.running = False
                break

        # Close OpenCV window after quitting
        cv2.destroyAllWindows()
        self.running = False
    def convert2rgb(self, img):
        R = img[:, :, 2]
        G = img[:, :, 1]
        B = img[:, :, 0]

        return R, G, B

    def threshold_binary(self, img, thresh=(200, 255)):
        binary = np.zeros_like(img)
        binary[(img >= thresh[0]) & (img <= thresh[1])] = 1

        return binary*255

    def process_image(self, img):
        """Image processing task."""
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0

        rows, cols = img.shape[:2]
        total_area = rows * cols

        
        R, G, B = self.convert2rgb(img)

        redMask = self.threshold_binary(R, (220, 255))
        stackedMask = np.dstack((redMask, redMask, redMask))
        contourMask = stackedMask.copy()
        crosshairMask = stackedMask.copy()

        # return value of findContours depends on OpenCV version
        (contours, hierarchy) = cv2.findContours(
            redMask.copy(), 1, cv2.CHAIN_APPROX_NONE
        )
        
        detection_msg = Bool()

        # Find the biggest contour (if detected)
        if len(contours) > 0:

            c = max(contours, key=cv2.contourArea)
            contour_area = cv2.contourArea(c)
            area_percentage = contour_area / total_area
            
            M = cv2.moments(c)

            # Make sure that "m00" won't cause ZeroDivisionError: float division by zero
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
            else:
                cx, cy = 0, 0

            # Show contour and centroid
            cv2.drawContours(contourMask, contours, -1, (0, 255, 0), 10)
            cv2.circle(contourMask, (cx, cy), 5, (0, 255, 0), -1)

            # Show crosshair and difference from middle point
            cv2.line(crosshairMask, (cx, 0), (cx, rows), (0, 0, 255), 10)
            cv2.line(crosshairMask, (0, cy), (cols, cy), (0, 0, 255), 10)
            cv2.line(
                crosshairMask,
                (int(cols / 2), 0),
                (int(cols / 2), rows),
                (255, 0, 0),
                10,
            )
            
            # Check if ball is close enough (based on area)
            if area_percentage >= self.area_threshold:
                # Ball is close enough - STOP
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                
                if not self.stopped_near_ball:
                    self.get_logger().info(f"Ball detected and close enough! Area: {area_percentage:.2%} - STOPPING")
                    self.stopped_near_ball = True
                    self.ball_detected = True
                    
                    # Publish object detected = True (state changed)
                    detection_msg.data = True
                    self.object_detected_pub.publish(detection_msg)
                
            else:
                # Ball detected but not close enough - Chase it
                if self.stopped_near_ball:
                    # State change: was stopped, now chasing again
                    self.get_logger().info(f"Ball moved away, chasing again... Area: {area_percentage:.2%}")
                    detection_msg.data = True
                    self.object_detected_pub.publish(detection_msg)
                
                self.stopped_near_ball = False
                
                if not self.ball_detected:
                    # State change: ball just appeared
                    self.get_logger().info(f"Ball detected! Starting chase... Area: {area_percentage:.2%}")
                    detection_msg.data = True
                    self.object_detected_pub.publish(detection_msg)
                    self.ball_detected = True
                
                # Chase the ball
                if abs(cols / 2 - cx) > 20:
                    msg.linear.x = 0.0
                    if cols / 2 > cx:
                        msg.angular.z = 0.2
                    else:
                        msg.angular.z = -0.2
                else:
                    msg.linear.x = 0.2
                    msg.angular.z = 0.0

        else:
            # No ball detected
            
            if self.ball_detected or self.stopped_near_ball:
                self.get_logger().info("Ball lost! Resuming exploration...")
                self.ball_detected = False
                self.stopped_near_ball = False
                
                # Publish object detected = False (ball lost - state changed)
                detection_msg.data = False
                self.object_detected_pub.publish(detection_msg)

        # Publish cmd_vel
        self.publisher.publish(msg)

        # Return processed frames
        return redMask, contourMask, crosshairMask

def main(args=None):

    print("OpenCV version: %s" % cv2.__version__)

    rclpy.init(args=args)
    node = ImageSubscriber()
    
    try:
        node.display_image()  # Run the display loop
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()