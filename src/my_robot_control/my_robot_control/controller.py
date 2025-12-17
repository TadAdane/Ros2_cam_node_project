import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        
        #publisher to move the robot
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        #subscriber to the camera (webcam or sim)
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10)
        
        self.bridge = CvBridge()
        self.get_logger().info("controller node started! click the window to move.")

    def image_callback(self, msg):
        #convert ROS Image to OpenCV Image
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        #display the image
        cv2.imshow("robot control interface", frame)
        
        #set mouse callback to detect clicks
        cv2.setMouseCallback("robot control interface", self.handle_click, param=frame.shape)
        
        cv2.waitKey(1)

    def handle_click(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            height, width, _ = param
            twist = Twist()
            
            #logic: click upper half -> forward, lower half -> backward
            if y < height / 2:
                self.get_logger().info("clicked UP - moving forward")
                twist.linear.x = 0.2
            else:
                self.get_logger().info("clicked DOWN - moving backward")
                twist.linear.x = -0.2
                
            self.publisher_.publish(twist)
            
            #stop after 1 second (safety)
            import time
            time.sleep(1.0)
            twist.linear.x = 0.0
            self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = RobotController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()
