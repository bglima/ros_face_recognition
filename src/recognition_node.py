#! /usr/bin/python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from face_detector import FaceDetector

class CameraNode:
    def __init__(self):
        self.cv2_img = None
        self.bridge = CvBridge()
        self.setup_ros_node()
        self.setup_blank_image()
        self.setup_detector()

    # Setup ROS node
    def setup_ros_node(self):
        rospy.init_node("recognition_node", anonymous=True)
        self.hertz = rospy.get_param('~hertz')
        self.node_name = rospy.get_param('~node_name')
        self.topic_name = rospy.get_param('~camera_topic')
        self.window_name = self.node_name + ':' + self.topic_name


    # Setup a start blank image_raw
    def setup_blank_image(self):
        # Create black image
        self.blank_img =  np.zeros((640, 480, 3), np.uint8)

        # Setup text
        font = cv2.FONT_HERSHEY_SIMPLEX
        text = "Waiting for video input..."

        # Get boundary of this text
        textsize = cv2.getTextSize(text, font, 1, 2)[0]

        # Get coords based on boundary
        textX = (self.blank_img.shape[1] - textsize[0]) / 2
        textY = (self.blank_img.shape[0] + textsize[1]) / 2

        # Add text centered on image
        cv2.putText(self.blank_img, text, (textX, textY ), font, 1, (255, 255, 255), 2)

        # Show
        cv2.imshow(self.window_name, self.blank_img)

    # Init and setup face detector in node
    def setup_detector(self):
        # Instantiate the object
        self.detector = FaceDetector()

        # Add new people to detector
        self.detector.add_to_database("Bruno Lima", "/home/brunolima/vision_ws/src/ros_face_recognition/media/train/bruno_lima/bruno_05.png", (255, 0, 0))
        self.detector.add_to_database("Joao Victor", "/home/brunolima/vision_ws/src/ros_face_recognition/media/train/joao_victor/joao_01.jpg", (0, 0, 255))


    # Shows an image that was published to
    def image_callback(self, msg):
        try:
            # Convert your ROS Image to OpenCV image
            self.cv2_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError, e:
            print(e)

    # Create and configure windows that will be used
    def start_node(self):
        cv2.namedWindow(self.window_name)

        # Init node and configurations
        rate = rospy.Rate(self.hertz)
        sub = rospy.Subscriber(self.topic_name, Image, self.image_callback)
        rospy.loginfo('Face recognition module is up and running!')

        while not rospy.is_shutdown():
            # If image contains anything
            if not (self.cv2_img is None):
                self.detector.process_frame( self.cv2_img )
                cv2.imshow(self.window_name, self.detector.draw_results())
            else:
                cv2.imshow(self.window_name, self.blank_img )

            # Check if q was pressed
            key = cv2.waitKey(5)
            if key == ord('q'):
                break
            try:
                rate.sleep()
            except KeyboardInterrupt:
                print('Shutting down...')
                cv2.destroyAllWindows()

def main():
    node = CameraNode()
    node.start_node()

if __name__ == '__main__':
    main()
