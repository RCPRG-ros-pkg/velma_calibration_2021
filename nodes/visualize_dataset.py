#!/usr/bin/env python

# Copyright (c) 2021, Robot Control and Pattern Recognition Group,
# Institute of Control and Computation Engineering
# Warsaw University of Technology
#
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Warsaw University of Technology nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYright HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Author: Dawid Seredynski
#

import roslib; roslib.load_manifest('velma_calibration')

import rospy
import copy
import math
import numpy as np
import PyKDL
import sys
from geometry_msgs.msg import Vector3

from rcprg_ros_utils import MarkerPublisher

from visualization_msgs.msg import Marker

from scipy import optimize

def printUsage():
    print('USAGE:\n visualize_dataset.py <csv_filename>')

class RobotModelTest:
    def __init__(self):
        self.__fixed_map = {}
        self.__dh_map = {}
        self.__parent_link_map = {}
        self.__parent_joint_map = {}
        self.__link_names = set()
        self.__joint_names = set()

        self.__poses_map = {}
        self.__config = {}

        self.__marker_pub = MarkerPublisher('obj_model')

    def getJointNames(self):
        return self.__joint_names

    def setConfig(self, config):
        self.__config = config
        self.__poses_map = {}

    def getAllPoses(self):
        for link_name in self.__link_names:
            self.getPose(link_name)
        return self.__poses_map

    def addFixed(self, joint_name, parent_link, child_link, pose):
        self.__fixed_map[child_link] = pose
        self.__parent_link_map[child_link] = parent_link
        self.__parent_joint_map[child_link] = joint_name
        self.__link_names.add(parent_link)
        self.__link_names.add(child_link)

    def addDH(self, joint_name, parent_link, child_link, d, a, alpha, dtheta ):
        self.__dh_map[child_link] = (d, a, alpha, dtheta)
        self.__parent_link_map[child_link] = parent_link
        self.__parent_joint_map[child_link] = joint_name
        assert not joint_name in self.__joint_names
        self.__joint_names.add(joint_name)
        self.__link_names.add(parent_link)
        self.__link_names.add(child_link)

    def getParentLinkName(self, link_name):
        if link_name in self.__parent_link_map:
            return self.__parent_link_map[link_name]
        # else:
        return None

    def getParentJointName(self, link_name):
        return self.__parent_joint_map[link_name]

    def getLocalTransform(self, link_name):
        if link_name in self.__fixed_map:
            return self.__fixed_map[link_name]
        # else:
        d, a, alpha, dtheta = self.__dh_map[link_name]
        theta = self.__config[self.getParentJointName(link_name)]
        #print('{}: {}'.format(self.getParentJointName(link_name), theta))
        return PyKDL.Frame(PyKDL.Vector(0, 0, d)) * PyKDL.Frame(PyKDL.Rotation.RotZ(theta+dtheta)) *\
                    PyKDL.Frame(PyKDL.Vector(a, 0, 0)) * PyKDL.Frame(PyKDL.Rotation.RotX(alpha))

    def getPose(self, link_name):
        if link_name in self.__poses_map:
            return self.__poses_map[link_name]
        # else:
        parent_link_name = self.getParentLinkName(link_name)
        if parent_link_name is None:
            return PyKDL.Frame()
        # else:

        parent_pose = self.getPose( parent_link_name )
        local_tf = self.getLocalTransform(link_name)
        pose = parent_pose * local_tf
        #print poseToURDFstr(local_tf)
        self.__poses_map[link_name] = pose
        return pose

def loadVelma():

    # 2021.09.27
    corrections = [
        ('torso_link0_right_arm_base_joint', -0.0004, -0.0007, -0.0000, -0.0003, -0.0006, -0.0006),
        ('right_arm_0_joint',-0.0001),
        ('right_arm_1_joint',-0.0027),
        ('right_arm_2_joint',-0.0013),
        ('right_arm_3_joint',-0.0018),
        ('right_arm_4_joint',-0.0029),
        ('right_arm_5_joint',-0.0218),
        ('right_arm_6_joint',-0.0000),
        ('right_needle_joint', 0.0011, -0.0008, 0.0006, 0, 0, 0),
        ('torso_link0_left_arm_base_joint', -0.0006, -0.0015, 0.0000, -0.0004, 0.0003, 0.0009),
        ('left_arm_0_joint', 0.0008),
        ('left_arm_1_joint', -0.0020),
        ('left_arm_2_joint', -0.0016),
        ('left_arm_3_joint', -0.0006),
        ('left_arm_4_joint', -0.0024),
        ('left_arm_5_joint', 0.0221),
        ('left_arm_6_joint', 0.0000),
        ('left_needle_joint', -0.0010, 0.0008, -0.0030, 0, 0, 0),
    ]

    #corrections = []

    # joint_name, parent_link, link, d, a, alpha, dtheta
    dh_params = [
        ('torso_0_joint', 'torso_base', 'torso_link0', 0.03, 0, 0, 0 ),

        ('right_arm_0_joint', 'calib_right_arm_base_link', 'right_arm_1_link', 0.3105, 0, -math.pi/2, 0 ),
        ('right_arm_1_joint', 'right_arm_1_link', 'right_arm_2_link', 0, 0, math.pi/2, 0 ),
        ('right_arm_2_joint', 'right_arm_2_link', 'right_arm_3_link', 0.4, 0, math.pi/2, 0 ),
        ('right_arm_3_joint', 'right_arm_3_link', 'right_arm_4_link', 0, 0, -math.pi/2, 0 ),
        ('right_arm_4_joint', 'right_arm_4_link', 'right_arm_5_link', 0.39, 0, -math.pi/2, 0 ),
        ('right_arm_5_joint', 'right_arm_5_link', 'right_arm_6_link', 0, 0, math.pi/2, 0 ),

        ('right_arm_6_joint', 'right_arm_6_link', 'right_arm_7_link', 0, 0, 0, 0 ),
        ('left_arm_0_joint', 'calib_left_arm_base_link', 'left_arm_1_link', 0.3105, 0, -math.pi/2, 0 ),
        ('left_arm_1_joint', 'left_arm_1_link', 'left_arm_2_link', 0, 0, math.pi/2, 0 ),
        ('left_arm_2_joint', 'left_arm_2_link', 'left_arm_3_link', 0.4, 0, math.pi/2, 0 ),
        ('left_arm_3_joint', 'left_arm_3_link', 'left_arm_4_link', 0, 0, -math.pi/2, 0 ),
        ('left_arm_4_joint', 'left_arm_4_link', 'left_arm_5_link', 0.39, 0, -math.pi/2, 0 ),
        ('left_arm_5_joint', 'left_arm_5_link', 'left_arm_6_link', 0, 0, math.pi/2, 0 ),
        ('left_arm_6_joint', 'left_arm_6_link', 'left_arm_7_link', 0, 0, 0, 0 ),
        ('head_pan_joint', 'head_pan_motor', 'head_pan_link', 0.11, 0, -math.pi/2, 0 ),
        ('head_tilt_joint', 'head_pan_link', 'head_tilt_link_dummy', 0, 0, math.pi, 0 ),
    ]

    # joint_name, parent_link, link, tx, ty, tz, rr, rp, ry, calibration_type
    fixed_params = [
        ('torso_base_joint', 'world', 'torso_base', 0, 0, 0, 0, 0, 0, None),
        ('head_base_joint', 'torso_link0', 'head_base', 0.020988, -0.00041436, 1.3117, 0.00040339, -0.0062724, -0.048944, 'pose'),
        ('head_pan_motor_joint', 'head_base', 'head_pan_motor', 0.0, 0.0, 0.025, 0, 0, 0, None),
        ('head_tilt_joint_dummy', 'head_tilt_link_dummy', 'head_tilt_link', 0, 0, 0, 0, 0, 0, None),
        ('stereo_left_joint', 'head_tilt_link', 'stereo_left_link', 0.013633, 0.22937, -0.045798, -1.5735, 0.013221, 0.023637, 'pose'),
        ('torso_link0_right_arm_base_joint', 'torso_link0', 'calib_right_arm_base_link', 0.0, -0.000188676, 1.17335, 0.0, -1.0471975512, math.pi/2, 'pose'),
        ('right_arm_ee_joint', 'right_arm_7_link', 'right_arm_ee_link', 0.0, 0.0, 0.078, 0, 0, 0, None),
        ('torso_link0_left_arm_base_joint', 'torso_link0', 'calib_left_arm_base_link', 0.0, 0.000188676, 1.17335, 0.0, 1.0471975512, math.pi/2, 'pose'),
        ('left_arm_ee_joint', 'left_arm_7_link', 'left_arm_ee_link', 0.0, 0.0, 0.078, 0, 0, 0, None),
        ('left_needle_joint', 'left_arm_ee_link', 'left_needle_link', 0.0, 0.0, 0.109, 0, 0, 0, 'translation'),
        ('right_needle_joint', 'right_arm_ee_link', 'right_needle_link', 0.0, 0.0, 0.112, 0, 0, 0, 'translation'),
    ]

    # Apply corrections
    dh_params_cor = []
    for dh in dh_params:
        found = False
        for corr in corrections:
            if dh[0] == corr[0]:
                dh_cor = (dh[0], dh[1], dh[2], dh[3], dh[4], dh[5], dh[6]+corr[1])
                dh_params_cor.append(dh_cor)
                found = True
                break
        if not found:
            dh_params_cor.append(dh)
    dh_params = dh_params_cor

    fixed_params_cor = []
    for fx in fixed_params:
        found = False
        for corr in corrections:
            if fx[0] == corr[0]:
                fx_cor = (fx[0], fx[1], fx[2], fx[3]+corr[1], fx[4]+corr[2], fx[5]+corr[3], fx[6]+corr[4], fx[7]+corr[5], fx[8]+corr[6], fx[9])
                fixed_params_cor.append(fx_cor)
                found = True
                break
        if not found:
            fixed_params_cor.append(fx)
    fixed_params = fixed_params_cor

    model = RobotModelTest()
    for joint_name, parent_link, link, d, a, alpha, dtheta in dh_params:
        model.addDH( joint_name, parent_link, link, d, a, alpha, dtheta )

    for joint_name, parent_link, link, tx, ty, tz, rr, rp, ry, calibration_type in fixed_params:
        model.addFixed(joint_name, parent_link, link,
                            PyKDL.Frame(PyKDL.Rotation.RPY(rr, rp, ry), PyKDL.Vector(tx, ty, tz)))

    return model

class Dataset:
    def __init__(self, csv_filename):
        with open(csv_filename, 'r') as f:
            lines = f.readlines()
        first_line = lines[0]
        if not first_line.startswith('#'):
            raise Exception('First line (comment) is missing')
        item_names = first_line[1:].split(',')
        self.item_names = []
        self.item_index_map = {}
        for idx, item_name in enumerate( item_names ):
            self.item_names.append( item_name.strip() )
            self.item_index_map[item_name.strip()] = idx
        self.points = []
        for line in lines[1:]:
            assert not line.startswith('#')
            point = []
            for item in line.split(','):
                data_item = item.strip()
                if not data_item:
                    continue
                point.append( float(data_item) )
            self.points.append(point)

    def getPointsCount(self):
        return len(self.points)

    def getItem(self, point_idx, item_name):
        return self.points[point_idx][self.item_index_map[item_name]]
'''
def printFrame(T):
    # With Quaternion
    q = T.M.GetQuaternion()
    print('PyKDL.Frame(PyKDL.Rotation.Quaternion({}, {}, {}, {}), PyKDL.Vector({}, {}, {}))'.format(
            q[0], q[1], q[2], q[3], T.p.x(), T.p.y(), T.p.z()))

    # With axis-angle
    #rot_v = T.M.GetRot()
    #angle = rot_v.Norm()
    #if angle < 0.00001:
    #    print('PyKDL.Frame(PyKDL.Vector({}, {}, {}))'.format(T.p.x(), T.p.y(), T.p.z()))
    #else:
    #    rot_v.Normalize()
    #    print('PyKDL.Frame(PyKDL.Rotation.Rot(PyKDL.Vector({}, {}, {}), {}), PyKDL.Vector({}, {}, {}))'.format(
    #            rot_v.x(), rot_v.y(), rot_v.z(), angle, T.p.x(), T.p.y(), T.p.z()))

def randomRotation(rot_sigma):
    while True:
        axis = PyKDL.Vector(random.normalvariate(0.0, 1.0),
                random.normalvariate(0.0, 1.0),
                random.normalvariate(0.0, 1.0))
        if axis.Norm() > 0.0001:
            break
    axis.Normalize()
    angle = random.normalvariate(0.0, rot_sigma)
    return PyKDL.Rotation.Rot(axis, angle)

def randomFrame(lin_sigma, rot_sigma):
    p = PyKDL.Vector(random.normalvariate(0.0, lin_sigma),
                        random.normalvariate(0.0, lin_sigma),
                        random.normalvariate(0.0, lin_sigma))
    return PyKDL.Frame(randomRotation(rot_sigma), p)

def frameToArray(T):
    rot_v = T.M.GetRot()
    return np.array([T.p.x(), T.p.y(), T.p.z(), rot_v.x(), rot_v.y(), rot_v.z()])

def arrayToFrame(arr):
    p = PyKDL.Vector(arr[0], arr[1], arr[2])
    rot_v = PyKDL.Vector(arr[3], arr[4], arr[5])
    angle = rot_v.Norm()
    rot_v.Normalize()
    return PyKDL.Frame(PyKDL.Rotation.Rot(rot_v, angle), p)

def meanPose(T_list):
    mean = None
    for T in T_list:
        if mean is None:
            mean = frameToArray(T)
        else:
            mean = mean + frameToArray(T)
    mean = mean / len(T_list)
    return arrayToFrame(mean)
'''

#def planeToArray(plane):

def estimatePlane(points_list):
    # There are outliers
    assert isinstance(points_list, list)

    def costFunc(x):
        norm = PyKDL.Rotation.RotX(x[0])*PyKDL.Rotation.RotY(x[1])*PyKDL.Vector(0,0,1)
        a = norm.x()
        b = norm.y()
        c = norm.z()
        #norm_len = a*a+b*b+c*c

        d = x[2]
        #est_T_W_O = arrayToFrame(x)
        result = []
        for pt in points_list:
            diff = pt.x()*a + pt.y()*b + pt.z()*c + d
            result.append(diff)
        return result

    # Prepare the initial estimations
    #mean_pose = meanPose(T_list)
    #mean_pos_object_markers = PyKDL.Vector()
    #mean_pos_world_markers = PyKDL.Vector()
    #count = 0
    #for marker_id, T_W_M in detected_markers.iteritems():
    #    T_O_M = object_markers[marker_id]
    #    mean_pos_object_markers = mean_pos_object_markers + T_O_M.p
    #    mean_pos_world_markers = mean_pos_world_markers + T_W_M.p
    #    count += 1
    #mean_pos_object_markers = mean_pos_object_markers / count
    #mean_pos_world_markers = mean_pos_world_markers / count
    #init_p = mean_pos_world_markers - mean_pos_object_markers
    #init_p = mean_pose.p
    #for it in range(200):
    if True:
        #init_M = randomRotation(math.radians(360.0))
        #init_T = PyKDL.Frame(init_M, init_p)
        init_plane = np.array([0, 0, 0])
        plane, ier = optimize.leastsq(costFunc, init_plane)
        #print('ier: {}'.format(ier))
        if ier in (1, 2, 3, 4):
            return plane
    return None

def main():
    if len(sys.argv) != 2:
        printUsage()
        return 1

    rospy.init_node('visualize_grasps', anonymous=False)
    rospy.sleep(0.5)

    csv_filename = sys.argv[1]

    dataset = Dataset(csv_filename)

    marker_pub = MarkerPublisher('calibration')

    model = loadVelma()
    while not rospy.is_shutdown():
        m_id = 0
        points_list = []
        for pt_idx in range(dataset.getPointsCount()):
            config = {}
            for joint_name in model.getJointNames():
                config[joint_name] = dataset.getItem(pt_idx, joint_name)

            model.setConfig(config)
            pose = model.getPose('right_needle_link')
            m_id = marker_pub.addVectorMarker(pose*PyKDL.Vector(0,0,-0.05), pose*PyKDL.Vector(), m_id, 1, 0, 0, a=1, frame='world', namespace='right', scale=0.001)
            m_id = marker_pub.addSinglePointMarker(pose.p, m_id, r=1, g=0, b=0, a=1, namespace='right', frame_id='world', m_type=Marker.SPHERE, scale=Vector3(0.002, 0.002, 0.002), T=None)
            points_list.append(pose.p)

            pose = model.getPose('left_needle_link')
            m_id = marker_pub.addVectorMarker(pose*PyKDL.Vector(0,0,-0.05), pose*PyKDL.Vector(), m_id, 0, 1, 0, a=1, frame='world', namespace='left', scale=0.001)
            m_id = marker_pub.addSinglePointMarker(pose.p, m_id, r=0, g=1, b=0, a=1, namespace='left', frame_id='world', m_type=Marker.SPHERE, scale=Vector3(0.002, 0.002, 0.002), T=None)

        plane = estimatePlane(points_list)
        if not plane is None:
            #print plane
            #nz = PyKDL.Vector(plane[0], plane[1], plane[2])
            nz = PyKDL.Rotation.RotX(plane[0])*PyKDL.Rotation.RotY(plane[1])*PyKDL.Vector(0,0,1)
            a = nz.x()
            b = nz.y()
            c = nz.z()
            plane_d = plane[2]

            error = 0.0
            for pt_idx in range(dataset.getPointsCount()):
                config = {}
                for joint_name in model.getJointNames():
                    config[joint_name] = dataset.getItem(pt_idx, joint_name)

                model.setConfig(config)
                pose = model.getPose('right_needle_link')
                error += (pose.p.x()*a + pose.p.y()*b + pose.p.z()*c + plane_d)**2
            print math.sqrt(error)

            if abs(nz.z()) > 0.7:
                nx = PyKDL.Vector(1,0,0)
            else:
                nx = PyKDL.Vector(0,0,1)
            ny = nz*nx
            nx = ny*nz
            ny.Normalize()
            nx.Normalize()

            T_W_P = PyKDL.Frame(PyKDL.Rotation(nx,ny,nz), -nz * plane_d)
            m_id = marker_pub.addSinglePointMarker(PyKDL.Vector(), m_id, r=1, g=0, b=0, a=0.5, namespace='right', frame_id='world', m_type=Marker.CUBE, scale=Vector3(2.0, 2.0, 0.001), T=T_W_P)

        marker_pub.publishAll()
        rospy.sleep(1)

    return 0

if __name__ == "__main__":
    exit(main())
