#!/usr/bin/env python
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


class CamTester(object):
    def __init__(self, camera_topic="/raspicam_node/image_raw"):

        (self.major, minor, _) = cv2.__version__.split(".")

        self.process_this_frame = True
        self.droped_frames = 0

        self.bridge_object = CvBridge()
        self.image_sub = rospy.Subscriber(camera_topic, Image, self.camera_callback)


    def camera_callback(self, data):

        self.process_this_frame = self.droped_frames >= 0

        if self.process_this_frame:
            self.droped_frames = 0
            try:
                cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
            except CvBridgeError as e:
                print(e)
                cv_image = None

            small_frame = cv2.resize(cv_image, (0, 0), fx=0.2, fy=0.2)

            height, width, channels = small_frame.shape

            cv2.imshow("Original", small_frame)
            cv2.waitKey(1)
        else:
            self.droped_frames += 1
            

    def loop(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('cam_image_test_start', anonymous=True)
    cam_tester = CamTester()
    cam_tester.loop()
