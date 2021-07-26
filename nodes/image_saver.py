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
    def __init__(self):
        self.trigger = False
        self.counter = 0

    def triggerCb(self, data):
        self.trigger = True
        print("Trigger")

    def saveImage(self, image, name):
        cv_image = self.bridge.imgmsg_to_cv2(image, desired_encoding='passthrough')
        fname = name + '_' + str(image.header.stamp.secs) + '_' + str(image.header.stamp.nsecs) + ".jpg"
        cv2.imwrite(fname, cv_image)
 

    def imageCb(self, image):
        if self.trigger:
            self.trigger = False
            self.saveImage(image, 'img')
            self.counter += 1
            self.pub.publish(self.counter)

    def stereoCb(self, left, right):
        if self.trigger:
            self.trigger = False
            self.saveImage(left, 'left')
            self.saveImage(right, 'right')
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
	self.stereo = rospy.get_param('~stereo', False)

        self.bridge = CvBridge()

        rospy.Subscriber("trigger", String, self.triggerCb)
        self.pub = rospy.Publisher("saved_images", Int32, queue_size = 10)

        if self.stereo:
            left_sub = Subscriber("left", Image)
            right_sub = Subscriber("right", Image)

            ats = ApproximateTimeSynchronizer([left_sub, right_sub], queue_size=5, slop=0.01)
            ats.registerCallback(self.stereoCb)
        else:
            rospy.Subscriber("image", Image, self.imageCb)

        rospy.spin()

def main():
    cs = ConfigSaver()
    cs.spin()

if __name__ == '__main__':
    main()
