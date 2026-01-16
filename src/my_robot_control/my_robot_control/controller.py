import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco #import the ArUco library
import numpy as np

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        
        #publisher to move the robot
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        #subscriber to usbcam
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10)
        
        self.bridge = CvBridge()
        
        # --- ARUCO SETUP ---
        #use a 4x4 dictionary (simple tags) on my phone
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.aruco_params = aruco.DetectorParameters()
        
        self.get_logger().info("ArUco Controller Started! Show Marker ID 0 to control.")

    def image_callback(self, msg):
        try:
            #convert ROS Image to OpenCV Image
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            height, width, _ = frame.shape
            
            #detect Markers
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, rejected = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
            
            twist = Twist()
            
            #logic
            if ids is not None:
                #draw a square around the marker
                aruco.drawDetectedMarkers(frame, corners, ids)
                
                #get the center of the first marker found (ID 0)
                # corners is a list of lists. corners[0] is the first marker
                #it has 4 points. We take the average to find the center
                c = corners[0][0] 
                center_x = int((c[0][0] + c[1][0] + c[2][0] + c[3][0]) / 4)
                center_y = int((c[0][1] + c[1][1] + c[2][1] + c[3][1]) / 4)
                
        
                cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)
                
            
                # screen origin (0,0) is Top-Left.
                # "Top Half" means y is SMALL. "Bottom Half" means y is BIG.
                
                if center_y < height / 2:
                    cv2.putText(frame, "MOVE FORWARD", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    twist.linear.x = 0.2
                else:
                    cv2.putText(frame, "MOVE BACKWARD", (50, height - 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                    twist.linear.x = -0.2
            else:
                #if no tag seen, stop moving
                twist.linear.x = 0.0

            #bublish  command
            self.publisher_.publish(twist)
            
            #center Line to divide the screen to the middle
            cv2.line(frame, (0, int(height/2)), (width, int(height/2)), (255, 255, 0), 2)
            
            #display
            cv2.imshow("ArUco Robot Control", frame)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = RobotController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()
