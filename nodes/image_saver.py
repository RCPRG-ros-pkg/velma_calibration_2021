#!/usr/bin/env python
# encoding: utf8

import rospy
from std_msgs.msg import String, Int32
from sensor_msgs.msg import Image
from message_filters import ApproximateTimeSynchronizer, Subscriber
from cv_bridge import CvBridge
import cv2

import copy
import sys
import os


class ConfigSaver:
    def __init__(self, filename):
        self.trigger = False
        self.counter = 0

    def triggerCb(self, data):
        self.trigger = True

    def gotimage(self, image):
        if self.trigger:
            self.trigger = False
            cv_image = self.bridge.imgmsg_to_cv2(image, desired_encoding='passthrough')
            fname = 'img_' + str(image.stamp.secs) + '_' + str(image.stamp.nsecs) + ".jpg"
            cv2.imwrite(fname, cv_image)
            self.counter += 1
            self.pub.publish(self.counter)


    def spin(self):
        # In ROS, nodes are uniquely named. If two nodes with the same
        # name are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.
        rospy.init_node('image_saver', anonymous=True)

        rospy.sleep(0.5)
        self.bridge = CvBridge()

        rospy.Subscriber("trigger", String, self.triggerCb)
        self.pub = rospy.Publisher("saved_positions", Int32)
        image_sub = Subscriber("/camera/image", Image)

        rospy.spin()

def main():
    cs = ConfigSaver(sys.argv[1])
    cs.spin()

if __name__ == '__main__':
    main()
