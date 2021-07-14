#!/usr/bin/env python
# encoding: utf8

import rospy
from std_msgs.msg import String

import copy
import sys
import os

from velma_common import VelmaInterface

class ConfigSaver:
    def __init__(self, filename):
        self.__filename = filename
        self.__joint_names = [
            "torso_0_joint",
            "right_arm_0_joint",
            "right_arm_1_joint",
            "right_arm_2_joint",
            "right_arm_3_joint",
            "right_arm_4_joint",
            "right_arm_5_joint",
            "right_arm_6_joint",
            "left_arm_0_joint",
            "left_arm_1_joint",
            "left_arm_2_joint",
            "left_arm_3_joint",
            "left_arm_4_joint",
            "left_arm_5_joint",
            "left_arm_6_joint",
            "head_pan_joint",
            "head_tilt_joint",
            "right_HandFingerOneKnuckleOneJoint",
            "right_HandFingerOneKnuckleTwoJoint",
            "right_HandFingerOneKnuckleThreeJoint",
            "right_HandFingerTwoKnuckleOneJoint",
            "right_HandFingerTwoKnuckleTwoJoint",
            "right_HandFingerTwoKnuckleThreeJoint",
            "right_HandFingerThreeKnuckleTwoJoint",
            "right_HandFingerThreeKnuckleThreeJoint",
            "left_HandFingerOneKnuckleOneJoint",
            "left_HandFingerOneKnuckleTwoJoint",
            "left_HandFingerOneKnuckleThreeJoint",
            "left_HandFingerTwoKnuckleOneJoint",
            "left_HandFingerTwoKnuckleTwoJoint",
            "left_HandFingerTwoKnuckleThreeJoint",
            "left_HandFingerThreeKnuckleTwoJoint",
            "left_HandFingerThreeKnuckleThreeJoint",
            "leftFtSensorJoint",
            "rightFtSensorJoint",
        ]
        with open(self.__filename, 'a') as f:
            str_line = '# time_s, time_ns, '
            for joint_name in self.__joint_names:
                str_line += '{}, '.format( joint_name )

            for marker in ['TL', 'LA', 'LB', 'LC', 'LD', 'RA', 'RB', 'RC', 'RD']:
                for coord in ['tx', 'ty', 'tz', 'rr', 'rp', 'ry']:
                    str_line += '{}.{}, '.format(marker, coord)
            f.write('{}\n'.format(str_line))
            #print(str_line)

    def triggerCb(self, data):
        print('trigger')
        stamped_js = self.__velma_interface.getLastJointState()
        if stamped_js is None:
            print('Could not read the current configuration')
        else:
            timestamp, js = stamped_js
            with open(self.__filename, 'a') as f:
                str_line = '{}, {}, '.format(timestamp.secs, timestamp.nsecs)
                for joint_name in self.__joint_names:
                    str_line += '{}, '.format( js[joint_name] )

                current_time = None#rospy.Time.now()
                timeout = 0.5
                T_HB_TL = self.__velma_interface.getTf('head_base', 'head_tilt_link', time=current_time, timeout_s=timeout)
                T_LP_LMA = self.__velma_interface.getTf('head_base', 'left_HandPalmMarkerALink', time=current_time, timeout_s=timeout)
                T_LP_LMB = self.__velma_interface.getTf('head_base', 'left_HandPalmMarkerBLink', time=current_time, timeout_s=timeout)
                T_LP_LMC = self.__velma_interface.getTf('head_base', 'left_HandPalmMarkerCLink', time=current_time, timeout_s=timeout)
                T_LP_LMD = self.__velma_interface.getTf('head_base', 'left_HandPalmMarkerDLink', time=current_time, timeout_s=timeout)

                T_RP_RMA = self.__velma_interface.getTf('head_base', 'right_HandPalmMarkerALink', time=current_time, timeout_s=timeout)
                T_RP_RMB = self.__velma_interface.getTf('head_base', 'right_HandPalmMarkerBLink', time=current_time, timeout_s=timeout)
                T_RP_RMC = self.__velma_interface.getTf('head_base', 'right_HandPalmMarkerCLink', time=current_time, timeout_s=timeout)
                T_RP_RMD = self.__velma_interface.getTf('head_base', 'right_HandPalmMarkerDLink', time=current_time, timeout_s=timeout)

                str_line += self.poseToStr(T_HB_TL)
                str_line += self.poseToStr(T_LP_LMA)
                str_line += self.poseToStr(T_LP_LMB)
                str_line += self.poseToStr(T_LP_LMC)
                str_line += self.poseToStr(T_LP_LMD)

                str_line += self.poseToStr(T_RP_RMA)
                str_line += self.poseToStr(T_RP_RMB)
                str_line += self.poseToStr(T_RP_RMC)
                str_line += self.poseToStr(T_RP_RMD)

                f.write('{}\n'.format(str_line))
                print(str_line)

            with open(self.__filename, 'r') as f:
                lines = f.readlines()

                print('Dataset size: {}'.format( len(lines)-1 ))

    def poseToStr(self, pose):
        rpy = pose.M.GetRPY()
        return '{}, {}, {}, {}, {}, {}, '.format(pose.p.x(), pose.p.y(), pose.p.z(), rpy[0], rpy[1], rpy[2])

    def spin(self):
        # In ROS, nodes are uniquely named. If two nodes with the same
        # name are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.
        rospy.init_node('config_saver', anonymous=True)

        rospy.sleep(0.5)

        # Create interface for Velma
        print('Creating VelmaInterface...')
        self.__velma_interface = VelmaInterface()

        print('Waiting for VelmaInterface initialization...')
        if not self.__velma_interface.waitForInit(timeout_s=10.0):
            exitError(3, msg='Could not initialize VelmaInterface')
        print('Initialization ok!')

        rospy.Subscriber("trigger", String, self.triggerCb)

        rospy.spin()

def main():
    cs = ConfigSaver(sys.argv[1])
    cs.spin()

if __name__ == '__main__':
    main()
