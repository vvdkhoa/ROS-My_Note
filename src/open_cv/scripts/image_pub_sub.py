#!/usr/bin/env python

import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import sys

# ROS Image message -> opencv message

# Note: run below node before using camera
# rosrun usb_cam usb_cam_node _pixel_format:=yuyv


bridge = CvBridge()

def image_callback(ros_image):

    print('Got an image')
    global bridge

    # Convert ros_image into an opencv-compatible image
    try:
        cv_image = bridge.imgmsg_to_cv2(ros_image, 'bgr8')
    except CvBridgeError as e:
        print(e)
    
    # Opencv code
    (rows, cols, channels) = cv_image.shape
    if cols > 200 and rows > 200:
        cv2.circle(cv_image, (100,100), 90, 255)
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(cv_image, 'Webcam Activate with ROS & OpenCV!', 
                (10,350), font, 1, (255,255,255),2,cv2.LINE_AA)
    cv2.imshow("Image window", cv_image)

    cv2.waitKey(3)
    # Opencv code end

def main(args):

    # Init ros
    rospy.init_node('image_converter', anonymous=True)

    # For turtlebot3 waffle: image_topic="/camera/rgb/image_raw/compressed"
    # For usb cam: image_topic="/usb_cam/image_raw"
    image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, image_callback)
    
    # Ros spin
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('Shutting down')

    # Destroy opencv
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
