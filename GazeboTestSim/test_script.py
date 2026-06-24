import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import cv2

bridge = CvBridge()

# Create subscriber for /rgb_camera topic
def rgb_camera_callback(msg):
    try:
        # Convert the ROS Image message to an OpenCV BGR image
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # Downscale image by a factor of 4
        #downscaled_image = cv2.resize(cv_image, (cv_image.shape[1] // 4, cv_image.shape[0] // 4))
        
        cv2.imshow("RGB Camera Image", cv_image)
        cv2.waitKey(10)  # Add a small delay to allow the image to be displayed properly

    except CvBridgeError as e:
        print("CvBridge Error: {0}".format(e))

    

# Create ROS node, publish to /cmd_vel topic, and subscribe to /rgb_camera topic
if __name__ == "__main__":
    rclpy.init()
    node = rclpy.create_node("test_node")
    
    # Create publisher for /cmd_vel topic
    vel_pub = node.create_publisher(Twist, "/cmd_vel", 10)
    
    vel_msg = Twist()
    vel_msg.angular.z = 0.5  # Set angular velocity around z-axis
    
    vel_pub.publish(vel_msg)

    rgb_camera_subscriber = node.create_subscription(Image, "/rgb_camera", rgb_camera_callback, 10)
    
    # Spin the node to keep it alive and processing callbacks
    rclpy.spin(node)
    
    # Clean up and shutdown
    node.destroy_node()
    rclpy.shutdown()