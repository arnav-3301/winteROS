#whole file is a change 
import rclpy
import math
import numpy as np
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Bool
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
from tf2_ros import TransformListener, Buffer
from tf2_geometry_msgs import do_transform_point
from geometry_msgs.msg import PointStamped


class BallPickupController(Node):
    def __init__(self):
        super().__init__('ball_pickup_controller')
        
        # Publisher for joint trajectories
        self.trajectory_pub = self.create_publisher(
            JointTrajectory, 
            '/arm_controller/joint_trajectory', 
            10
        )
        
        # Publisher for picked object confirmation
        self.picked_pub = self.create_publisher(
            Bool,
            '/arm_controller/picked_object',
            10
        )
        
        # Subscriber for object detection trigger
        self.detection_sub = self.create_subscription(
            Bool,
            '/arm_controller/found_object',
            self.detection_callback,
            10
        )
        
        # Subscribers for depth camera
        self.depth_sub = self.create_subscription(
            Image,
            '/camera/depth_image',
            self.depth_callback,
            10
        )
        
        self.rgb_sub = self.create_subscription(
            Image,
            '/camera/image',
            self.rgb_callback,
            10
        )
        
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.camera_info_callback,
            10
        )
        
        # TF2 for coordinate transformation
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # CV Bridge for image conversion
        self.bridge = CvBridge()
        
        # State variables
        self.depth_image = None
        self.rgb_image = None
        self.camera_info = None
        self.pickup_in_progress = False
        
        self.get_logger().info('Ball Pickup Controller initialized')
    
    def camera_info_callback(self, msg):
        """Store camera intrinsic parameters"""
        self.camera_info = msg
    
    def depth_callback(self, msg):
        """Store latest depth image"""
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f'Error converting depth image: {e}')
    
    def rgb_callback(self, msg):
        """Store latest RGB image"""
        try:
            self.rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Error converting RGB image: {e}')
    
    def detection_callback(self, msg):
        """Triggered when object is detected"""
        if msg.data and not self.pickup_in_progress:
            self.get_logger().info('Object detected! Starting pickup sequence...')
            # TODO:
            # < Need to change one variable what is that >
            self.pickup_in_progress = True
            self.execute_pickup()
            # < In case the picking up fails and comes outside we want to tell the process we finished our work 
            # (unsuccesfully) (doesn't matter succesfuly or unsuccesfully just msg should be same I finished my work)>
            # END TODO
            
            
    def find_ball_centroid(self):
        """
        Detect red ball in RGB image and get its 3D coordinates
        Returns: [x, y, z] in camera frame or None if not found
        """
        if self.rgb_image is None or self.depth_image is None or self.camera_info is None:
            self.get_logger().warn('Missing camera data')
            return None
        
        # Convert to HSV for red color detection
        hsv = cv2.cvtColor(self.rgb_image, cv2.COLOR_BGR2HSV)
        
        # Red color range (red wraps around in HSV, so we need two ranges)
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([180, 255, 255])
        
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            self.get_logger().warn('No red ball detected in image')
            return None
        
        # Get largest contour (assumed to be the ball)
        largest_contour = max(contours, key=cv2.contourArea)
        
        # Calculate centroid
        M = cv2.moments(largest_contour)
        if M['m00'] == 0:
            return None
        
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        
        self.get_logger().info(f'Ball centroid in image: ({cx}, {cy})')
        
        # Get depth at centroid
        
        self.get_logger().info(f'RAW DEPTH: {self.depth_image[cy, cx]}')
        
        depth = self.depth_image[cy, cx]
        
        if depth == 0 or np.isnan(depth):
            self.get_logger().warn('Invalid depth value')
            return None
        
        # Convert pixel coordinates to 3D camera coordinates
        fx = self.camera_info.k[0]
        fy = self.camera_info.k[4]
        cx_cam = self.camera_info.k[2]
        cy_cam = self.camera_info.k[5]
        
        # Camera frame coordinates
        x_cam = (cx - cx_cam) * depth / fx
        y_cam = (cy - cy_cam) * depth / fy
        z_cam = depth
        
        self.get_logger().info(f'Ball position in camera frame: ({x_cam:.3f}, {y_cam:.3f}, {z_cam:.3f})')
        
        return [x_cam, y_cam, z_cam]
    
    def transform_to_base(self, point_camera):
        try:
            # 1. Create the PointStamped
            point_stamped = PointStamped()
            point_stamped.header.frame_id = 'camera_link_optical'
            point_stamped.header.stamp = self.get_clock().now().to_msg()
            point_stamped.point.x = point_camera[0]
            point_stamped.point.y = point_camera[1]
            point_stamped.point.z = point_camera[2]
            
            # 2. Lookup the transform
            transform = self.tf_buffer.lookup_transform(
                'arm_base_link',
                'camera_link_optical',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            
            # 3. USE THE LIBRARY TO TRANSFORM (This handles the rotation!)
            point_base = do_transform_point(point_stamped, transform)
            
            # 4. Extract the results
            x_base = point_base.point.x
            y_base = point_base.point.y
            z_base = point_base.point.z
            
            self.get_logger().info(f'Ball position in base frame: ({x_base:.3f}, {y_base:.3f}, {z_base:.3f})')
            
            return [x_base, y_base, z_base]
            
        except Exception as e:
            self.get_logger().error(f'Transform failed: {e}')
            return None
    
    def execute_pickup(self):
        """Execute the complete pickup sequence"""
        # Find ball in camera frame
        ball_camera = self.find_ball_centroid()
        import time
        if ball_camera is None:
            self.get_logger().error('Failed to detect ball')
            self.pickup_in_progress = False

            done = Bool()
            done.data = True
            self.picked_pub.publish(done)
            return
        
        # Transform to base frame
        ball_base = self.transform_to_base(ball_camera)
        
        if ball_base is None:
            self.get_logger().error('Failed to transform coordinates')
            self.pickup_in_progress = False
            done = Bool()
            done.data = True
            self.picked_pub.publish(done)
            return
        
        x, y, z = ball_base
        
        #TODO:
        
        # <x y z are the cylinder cordinates now use those coordinates experiment around and make sure the arm is picking up the cylinder >
        # <If any motion is happening to fast remember to change duration>
        # Cylinder geometry
        CYLINDER_LENGTH = 0.254  # meters (10 inches)
        COM_OFFSET = CYLINDER_LENGTH / 2.0

        # Compute grasp pose at center of mass
        grasp_x = x + 0.8
        grasp_y = y
        grasp_z = z - COM_OFFSET   # CRITICAL FIX

        # Motion heights
        approach_z = grasp_z + 0.18
        lift_z = grasp_z + 0.25

        self.get_logger().info("Step 1: Approach above COM")
        self.publish_trajectory(
            grasp_x, grasp_y, approach_z,
            "open", 2.0,
            gripper_angle=math.pi/2   # CRITICAL FIX
        )

        self.get_logger().info("Step 2: Descend to COM")
        self.publish_trajectory(
            grasp_x, grasp_y, grasp_z,
            "open", 1.5,
            gripper_angle=math.pi/2
        )

        self.get_logger().info("Step 3: Close gripper at COM")
        self.publish_trajectory(
            grasp_x, grasp_y, grasp_z,
            "closed", 0.6,
            gripper_angle=math.pi/2
        )

        self.get_logger().info("Step 4: Lift slowly")
        self.publish_trajectory(
            grasp_x, grasp_y, lift_z,
            "closed", 4.0,
            gripper_angle=math.pi/2
        )

        self.get_logger().info("Step 5: Retreat")
        self.publish_trajectory(
            0.25, 0.0, lift_z,
            "closed", 3.0,
            gripper_angle=math.pi/2
        )

        
        
        done = Bool()
        done.data = True
        self.picked_pub.publish(done)

        self.pickup_in_progress = False
        # Publish completion message
        # <We finished picking up the ball succesfully so publish message (guess which topic )>
        # <Again need to change one variable what was it ???>
        #END TODO
        
        self.get_logger().info('Pickup sequence complete!')
        
    
    def publish_trajectory(self, x, y, z, gripper_status, duration):
        """Publish a single trajectory point and wait for completion"""
        import time
        
        trajectory = JointTrajectory()
        trajectory.joint_names = [
            'shoulder_pan_joint', 
            'shoulder_lift_joint', 
            'elbow_joint', 
            'wrist_joint', 
            'left_finger_joint', 
            'right_finger_joint'
        ]
        #TODO:
        
        #<Check inverse_kinematics.py from week 4 check how we defined points and what would be the equivalent code here>
        point = JointTrajectoryPoint()

        angles = self.inverse_kinematics([x, y, z], gripper_status)

        point.positions = angles
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration % 1) * 1e9)

        # END TODO
        trajectory.points = [point]
        
        self.trajectory_pub.publish(trajectory)
        self.get_logger().info(f'Published trajectory to ({x:.3f}, {y:.3f}, {z:.3f}), gripper: {gripper_status}')
        
        # Wait for the trajectory to complete (duration + small buffer)
        time.sleep(duration + 0.5)
        self.get_logger().info(f'Trajectory completed, waited {duration + 0.5}s')
    
    def inverse_kinematics(self, coords, gripper_status, gripper_angle = 0):
        '''
        Calculates the joint angles according to the desired TCP coordinate and gripper angle
        :param coords: list, desired [X, Y, Z] TCP coordinates
        :param gripper_status: string, can be `closed` or `open`
        :param gripper_angle: float, gripper angle in woorld coordinate system (0 = horizontal, pi/2 = vertical)
        :return: list, the list of joint angles, including the 2 gripper fingers
        '''
        # link lengths
        ua_link = 0.2
        fa_link = 0.25
        tcp_link = 0.175
        # z offset (robot arm base height)
        z_offset = 0.1
        # default return list
        angles = [0,0,0,0,0,0]

        # Calculate the shoulder pan angle from x and y coordinates
        j0 = math.atan(coords[1]/coords[0])

        # Re-calculate target coordinated to the wrist joint (x', y', z')
        x = coords[0] - tcp_link * math.cos(j0) * math.cos(gripper_angle)
        y = coords[1] - tcp_link * math.sin(j0) * math.cos(gripper_angle)
        z = coords[2] - z_offset + math.sin(gripper_angle) * tcp_link

        # Solve the problem in 2D using x" and z'
        x = math.sqrt(y*y + x*x)

        # Let's calculate auxiliary lengths and angles
        c = math.sqrt(x*x + z*z)
        alpha = math.asin(z/c)
        beta = math.pi - alpha
        # Apply law of cosines
        gamma = math.acos((ua_link*ua_link + c*c - fa_link*fa_link)/(2*c*ua_link))

        j1 = math.pi/2.0 - alpha - gamma
        j2 = math.pi - math.acos((ua_link*ua_link + fa_link*fa_link - c*c)/(2*ua_link*fa_link)) # j2 = 180 - j2'
        delta = math.pi - (math.pi - j2) - gamma # delta = 180 - j2' - gamma

        j3 = math.pi + gripper_angle - beta - delta

        angles[0] = j0
        angles[1] = j1
        angles[2] = j2
        angles[3] = j3

        if gripper_status == "open":
            angles[4] = 0.04
            angles[5] = 0.04
        elif gripper_status == "closed":
            angles[4] = 0.01
            angles[5] = 0.01
        else:
            angles[4] = 0.04
            angles[5] = 0.04

        return angles


def main(args=None):
    rclpy.init(args=args)
    node = BallPickupController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()