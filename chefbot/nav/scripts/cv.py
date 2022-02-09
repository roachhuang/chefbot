#! /usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

class ShowingImage(object):
    def __init__(self):
        self.image.sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.cameraCb)
        self.bridge_object = CvBridge()

    def cameraCb(self, data):
        try:
            cv_image = self.bridge_object.imgmsg_tocv2(data, desired_encoding='bgr8')
        except CvBridgeError as e:
            print(e)
        
        cv3.imshow('image', cv_image)
        cv3.waitKey(0)

def main():
    showing_image_object = ShowingImage()
    rospy.init_node('line_following_node', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('shutting down')
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
