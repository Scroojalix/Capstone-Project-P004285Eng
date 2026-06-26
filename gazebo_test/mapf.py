import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import cv2

bridge = CvBridge()

class MAPFNode(Node):
    def __init__(self):
        super().__init__('mapf_whca_node')
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.rgb_camera_subscriber = self.create_subscription(Image, '/rgb_camera', self.rgb_camera_callback, 10)
        
    def rgb_camera_callback(self, msg):
        try:
            # Convert the ROS Image message to an OpenCV BGR image
            cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Downscale image by a factor of 4
            #downscaled_image = cv2.resize(cv_image, (cv_image.shape[1] // 4, cv_image.shape[0] // 4))
            
            #cv2.imshow("RGB Camera Image", cv_image)
            cv2.waitKey(10)  # Add a small delay to allow the image to be displayed properly

        except CvBridgeError as e:
            print("CvBridge Error: {0}".format(e))

    def publish_velocity(self):
        vel_msg = Twist()
        vel_msg.angular.z = 0.5  # Set angular velocity around z-axis
        self.vel_pub.publish(vel_msg)

# Create ROS node, publish to /cmd_vel topic, and subscribe to /rgb_camera topic
def main():
    rclpy.init()
    node = MAPFNode()
    
    node.get_logger().info("MAPFNode has been started. Publishing to /cmd_vel and subscribing to /rgb_camera.")
    #node.publish_velocity()  # Publish velocity command once at startup
    
    # Wait 5 seconds for ROS GZ bridge to establish the connection
    node.get_logger().info("Waiting for 5 seconds to allow ROS GZ bridge to establish the connection...")
    rclpy.spin_once(node, timeout_sec=5.0)
    
    node.publish_velocity()  # Publish velocity command after the wait
    
    # Spin the node to keep it alive and processing callbacks
    rclpy.spin(node)
    
    # Clean up and shutdown
    node.destroy_node()
    rclpy.shutdown()