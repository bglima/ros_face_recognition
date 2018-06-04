#! /usr/bin/python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

# Shows an image that was published to
def image_callback(msg):
    global cv2_img
    try:
        # Convert your ROS Image to OpenCV image
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError, e:
        print(e)

# Create and configure windows that will be used
def create_window():
    cv2.namedWindow(topic_name)

# Global variables
cv2_img = None
bridge = CvBridge()
hertz = 30
node_name = 'recognition_node'
topic_name = '/camera/image_raw'

def main():
    create_window()
    rospy.init_node(node_name)
    rate = rospy.Rate(hertz)
    sub = rospy.Subscriber(topic_name, Image, image_callback)
    rospy.loginfo('Face recognition module is up and running!')

    while not rospy.is_shutdown():
        # If image contains anything
        if not (cv2_img is None):
            cv2.imshow(topic_name, cv2_img)

        # Check if q was pressed
        key = cv2.waitKey(5)
        if key == ord('q'):
            break
        try:
            rate.sleep()
        except KeyboardInterrupt:
            print('Shutting down...')
            cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
