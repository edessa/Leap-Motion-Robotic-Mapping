#!/usr/bin/env python
__author__ = 'flier'
import rospy
import leap_interface
from leap_motion.msg import leap
from leap_motion.msg import leapros
import argparse
import numpy as np
import rospy
import math
from roslib import message
import cv2
from cv_bridge import CvBridge, CvBridgeError
import baxter_interface
import baxter_external_devices
from cv_bridge import CvBridge, CvBridgeError
from numpy.linalg import *
from numpy import linalg
from numpy import *
from scipy import linalg, matrix
import scipy
import ctypes
from ctypes import *
import openravepy
from openravepy import *
from numpy.ctypeslib import ndpointer
import time
import tf
import geometry_msgs.msg
from geometry_msgs.msg import (Point, PointStamped)
from std_msgs.msg import Header
import igraph
from igraph import *
import tf2_ros
from std_msgs.msg import String
import scipy.spatial
import array
from array import *
import multiprocessing
import operator
import random
from baxter_interface import CHECK_VERSION
from std_msgs.msg import Float64
from ar_track_alvar_msgs.msg import AlvarMarker, AlvarMarkers
import pcl_ros

# Obviously, this method publishes the data defined in leapros.msg to /leapmotion/data
def sender():
    li = leap_interface.Runner()
    li.setDaemon(True)
    li.start()
    # pub     = rospy.Publisher('leapmotion/raw',leap)
    pub_ros   = rospy.Publisher('leapmotion/data',leapros)
    rospy.init_node('leap_pub')
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled
    rs.enable()
    euler = tf.transformations.euler_from_quaternion([0.65, -0.27, 0.65, 0.273])    
    output = ctypes.CDLL('/home/edessale/ros_ws/src/openrave/python/baxIK.so')
    output.compute.restype = POINTER(c_float)
    output.compute.argtypes = [c_float, c_float, c_float, c_float, c_float, c_float, c_float]
   # res = output.compute(0.5, 0.5, 0.5, 0.65, -0.27, 0.65, 0.273)
    output2 = ctypes.CDLL('/home/edessale/ros_ws/src/openrave/python/baxIK2.so')
    output2.compute.restype = POINTER(c_float)
    output2.compute.argtypes = [c_float, c_float, c_float, c_float, c_float, c_float, c_float]
    env = openravepy.Environment()
    env.StopSimulation()
    env.Load('/home/edessale/ros_ws/src/openrave/python/baxterXMLs/Arms.xml')
    robot = env.GetRobots()[0]
    manip = robot.SetActiveManipulator('right_arm')
    manip2 = robot.SetActiveManipulator('left_arm')
    left = baxter_interface.Limb('left') #Working with left arm
    right = baxter_interface.Limb('right')
    left.set_joint_position_speed(0.6)
    right.set_joint_position_speed(0.6)
    lj = left.joint_angles() #array containing s0,s1,e0,e1,... of joint names
    rj = right.joint_angles()
    xStartLeft = 0.17
    yStartLeft = 0
    zStartLeft = -0.1

    xStartRight = 0.17
    yStartRight = -1
    zStartRight = -0.1


    LeapStartZ = 150 #Vert
    LeapEndZ = -100

    LeapEndX = 200 #Side
    LeapStartX = -200

    LeapEndY = 400 #UP
    LeapStartY = 20
    
    tros = tf.TransformListener()
    tros.setUsingDedicatedThread(True)

    tros2 = tf.TransformListener()
    tros2.setUsingDedicatedThread(True)
    count = 0
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
	count += 1
        hand_palm_pos_    = li.get_hand_palmpos()
        hand_pitch_       = li.get_hand_pitch()
        hand_roll_        = li.get_hand_roll()
        hand_yaw_         = li.get_hand_yaw()

	if li.getNumHands() == 2:
            hand_palm_pos_2    = li.get_hand_palmpos2()
            hand_pitch_2       = li.get_hand_pitch2()
            hand_roll_2        = li.get_hand_roll2()
            hand_yaw_2        = li.get_hand_yaw2()
	    x2Send = (hand_palm_pos_[2] - LeapStartZ) * 0.8/(LeapEndZ - LeapStartZ) + xStartRight
	    y2Send = (hand_palm_pos_[0] - LeapStartX) * 0.8/(LeapEndX - LeapStartX) + yStartRight
	    z2Send = (hand_palm_pos_[1] - LeapStartY) * 0.8/(LeapEndY - LeapStartY) + zStartRight
	    quaternion = tf.transformations.quaternion_from_euler(hand_pitch_, hand_roll_, hand_yaw_)
	    rightPt = geometry_msgs.msg.Point(x2Send, y2Send, z2Send) 
            tros.waitForTransform('/base','/right_arm_mount',rospy.Time(), rospy.Duration(2.0))
            t =  tros.lookupTransform('/base','/right_arm_mount',rospy.Time())
	    hdr = Header(stamp=rospy.Time(), frame_id='/base')
	    rightPt2 = geometry_msgs.msg.PointStamped(header=hdr, point=rightPt)
            result = tros.transformPoint('/right_arm_mount', rightPt2) 
	    print "right"
	    print rightPt
	    res = output2.compute(result.point.x, result.point.y, result.point.z, quaternion[0], quaternion[1], quaternion[2], quaternion[3])
	    results = np.ctypeslib.as_array(res, shape=(int(res[999]/7),7))
	    k = int(res[999]/7)
	    if k > 100:
	        k = 100
	    aTruth = False
	    z = 0
	    if k > 10:
	        while aTruth is False:
		    if z == k:
			aTruth = True
		    	break
		    a = results[z]
		    z += 1
		    robot.SetDOFValues(a, manip.GetArmIndices())
		    T = manip.GetEndEffectorTransform()
		    xSub = math.fabs(T[0][3] - rightPt2.point.x)
		    ySub = math.fabs(T[1][3] - rightPt2.point.y)
		    zSub = math.fabs(T[2][3] - rightPt2.point.z)
		    if xSub <= 0.2 and ySub <= 0.2 and zSub <= 0.2:
		        if checkJoints(a) is True and robot.CheckSelfCollision() is False:
		            array1 = [rj['right_s0'], rj['right_s1'], rj['right_e0'], rj['right_e1'], rj['right_w0'], rj['right_w1'], rj['right_w2']]
			    alpha = a[0:5]-array1[0:5]
			    min2 = numpy.linalg.norm(alpha)
			    if math.fabs(min2) < 5:
			        aTruth = True
		    	        print a
			        rj['right_s0']= a[0]
			        rj['right_s1']= a[1]
			        rj['right_e0']= a[2]
			        rj['right_e1']= a[3]
			        rj['right_w0']= a[4]
			        rj['right_w1']= a[5]
			        rj['right_w2']= a[6]
			        print "MOVING"
			        #right.move_to_joint_positions(rj, timeout=20, threshold = 0.1)
				right.set_joint_positions(rj)
			        time.sleep(0.3)

	    x2Send = (hand_palm_pos_2[2] - LeapStartZ) * 0.8/(LeapEndZ - LeapStartZ) + xStartLeft
	    y2Send = (hand_palm_pos_2[0] - LeapStartX) * 0.8/(LeapEndX - LeapStartX) + yStartLeft
	    z2Send = (hand_palm_pos_2[1] - LeapEndY) * 0.8/(LeapStartY - LeapEndY) + zStartLeft
	    quaternion = tf.transformations.quaternion_from_euler(hand_pitch_2, hand_roll_2, hand_yaw_2)
	    leftPt = geometry_msgs.msg.Point(x2Send, y2Send, z2Send) 
            tros.waitForTransform('/base','/left_arm_mount',rospy.Time(), rospy.Duration(2.0))
            t =  tros.lookupTransform('/base','/left_arm_mount',rospy.Time())
	    hdr = Header(stamp=rospy.Time(), frame_id='/base')
	    leftPt2 = geometry_msgs.msg.PointStamped(header=hdr, point=leftPt)
            result = tros.transformPoint('/left_arm_mount', leftPt2) 
	    print "left"
	    print leftPt
	    res = output2.compute(result.point.x, result.point.y, result.point.z, quaternion[0], quaternion[1], quaternion[2], quaternion[3])
	    results = np.ctypeslib.as_array(res, shape=(int(res[999]/7),7))
	    k = int(res[999]/7)
	    if k > 100:
	        k = 100
	    aTruth = False
	    z = 0
	    if k > 10:
	        while aTruth is False:
		    if z == k:
			aTruth = True
		    	break
		    a = results[z]
		    z += 1
		    robot.SetDOFValues(a, manip2.GetArmIndices())
		    T = manip2.GetEndEffectorTransform()
		    xSub = math.fabs(T[0][3] - leftPt2.point.x)
		    ySub = math.fabs(T[1][3] - leftPt2.point.y)
		    zSub = math.fabs(T[2][3] - leftPt2.point.z)
		    if xSub <= 0.2 and ySub <= 0.2 and zSub <= 0.2:
		        if checkJoints(a) is True and robot.CheckSelfCollision() is False:
		            array1 = [lj['left_s0'], lj['left_s1'], lj['left_e0'], lj['left_e1'], lj['left_w0'], lj['left_w1'], lj['left_w2']]
			    alpha = array1[0:5] - a[0:5]
			    min2 = numpy.linalg.norm(alpha)
			    if math.fabs(min2) < 3:
			        aTruth = True
		    	        print a
			        lj['left_s0']= a[0]
			        lj['left_s1']= a[1]
			        lj['left_e0']= a[2]
			        lj['left_e1']= a[3]
			        lj['left_w0']= a[4]
			        lj['left_w1']= a[5]
			        lj['left_w2']= a[6]
			        print "MOVING"
				left.set_joint_positions(lj)
			        #left.move_to_joint_positions(lj, timeout=20, threshold = 0.1)
		



	if li.getNumHands() == 1:
	    x2Send = (hand_palm_pos_[2] - LeapStartZ) * 0.8/(LeapEndZ - LeapStartZ) + xStartRight
	    y2Send = (hand_palm_pos_[0] - LeapStartX) * 0.8/(LeapEndX - LeapStartX) + yStartRight
	    z2Send = (hand_palm_pos_[1] - LeapStartY) * 0.8/(LeapEndY - LeapStartY) + zStartRight
	    quaternion = tf.transformations.quaternion_from_euler(hand_pitch_, hand_roll_, hand_yaw_)
	    rightPt = geometry_msgs.msg.Point(x2Send, y2Send, z2Send) 
            tros.waitForTransform('/base','/right_arm_mount',rospy.Time(), rospy.Duration(2.0))
            t =  tros.lookupTransform('/base','/right_arm_mount',rospy.Time())
	    hdr = Header(stamp=rospy.Time(), frame_id='/base')
	    rightPt2 = geometry_msgs.msg.PointStamped(header=hdr, point=rightPt)
            result = tros.transformPoint('/right_arm_mount', rightPt2) 
	    print "right"
	    print rightPt
	    res = output2.compute(result.point.x, result.point.y, result.point.z, quaternion[0], quaternion[1], quaternion[2], quaternion[3])
	    results = np.ctypeslib.as_array(res, shape=(int(res[999]/7),7))
	    k = int(res[999]/7)
	    if k > 100:
	        k = 100
	    aTruth = False
	    z = 0
	    if k > 10:
	        while aTruth is False:
		    if z == k:
			aTruth = True
		    	break
		    a = results[z]
		    z += 1
		    robot.SetDOFValues(a, manip.GetArmIndices())
		    T = manip.GetEndEffectorTransform()
		    xSub = math.fabs(T[0][3] - rightPt2.point.x)
		    ySub = math.fabs(T[1][3] - rightPt2.point.y)
		    zSub = math.fabs(T[2][3] - rightPt2.point.z)
		    if xSub <= 0.2 and ySub <= 0.2 and zSub <= 0.2:
		        if checkJoints(a) is True and robot.CheckSelfCollision() is False:
		            array1 = [rj['right_s0'], rj['right_s1'], rj['right_e0'], rj['right_e1'], rj['right_w0'], rj['right_w1'], rj['right_w2']]
			    alpha = a[0:5]-array1[0:5]
			    min2 = numpy.linalg.norm(alpha)
			    if math.fabs(min2) < 5:
			        aTruth = True
		    	        print a
			        rj['right_s0']= a[0]
			        rj['right_s1']= a[1]
			        rj['right_e0']= a[2]
			        rj['right_e1']= a[3]
			        rj['right_w0']= a[4]
			        rj['right_w1']= a[5]
			        rj['right_w2']= a[6]
			        print "MOVING"
				right.set_joint_positions(rj)
			       # right.move_to_joint_positions(rj, timeout=20, threshold = 0.1)
			        time.sleep(0.3)

	if li.getNumHands() == 0:
	    x2Send = (hand_palm_pos_[2] - LeapStartZ) * 0.8/(LeapEndZ - LeapStartZ) + xStartLeft
	    y2Send = (hand_palm_pos_[0] - LeapStartX) * 0.8/(LeapEndX - LeapStartX) + yStartLeft
	    z2Send = (hand_palm_pos_[1] - LeapStartY) * 0.8/(LeapEndY - LeapStartY) + zStartLeft
	    quaternion = tf.transformations.quaternion_from_euler(hand_pitch_, hand_roll_, hand_yaw_)
	    leftPt = geometry_msgs.msg.Point(x2Send, y2Send, z2Send) 
            tros.waitForTransform('/base','/left_arm_mount',rospy.Time(), rospy.Duration(2.0))
            t =  tros.lookupTransform('/base','/left_arm_mount',rospy.Time())
	    hdr = Header(stamp=rospy.Time(), frame_id='/base')
	    leftPt2 = geometry_msgs.msg.PointStamped(header=hdr, point=leftPt)
            result = tros.transformPoint('/left_arm_mount', leftPt2) 

	    res = output2.compute(result.point.x, result.point.y, result.point.z, quaternion[0], quaternion[1], quaternion[2], quaternion[3])
	    results = np.ctypeslib.as_array(res, shape=(int(res[999]/7),7))
	    k = int(res[999]/7)
	    print "left"
	    print leftPt
	    if k > 100:
	        k = 100
	    aTruth = False
	    z = 0
	    if k > 10:
	        while aTruth is False:
		    if z == k:
			aTruth = True
		    	break
		    a = results[z]
		    z += 1
		    robot.SetDOFValues(a, manip2.GetArmIndices())
		    T = manip2.GetEndEffectorTransform()
		    xSub = math.fabs(T[0][3] - leftPt2.point.x)
		    ySub = math.fabs(T[1][3] - leftPt2.point.y)
		    zSub = math.fabs(T[2][3] - leftPt2.point.z)
		    if xSub <= 0.2 and ySub <= 0.2 and zSub <= 0.2:
		        if checkJoints(a) is True and robot.CheckSelfCollision() is False:
		            array1 = [lj['left_s0'], lj['left_s1'], lj['left_e0'], lj['left_e1'], lj['left_w0'], lj['left_w1'], lj['left_w2']]
			    alpha = array1[0:5] - a[0:5]
			    min2 = numpy.linalg.norm(alpha)
			    if math.fabs(min2) < 3:
			        aTruth = True
		    	        print a
			        lj['left_s0']= a[0]
			        lj['left_s1']= a[1]
			        lj['left_e0']= a[2]
			        lj['left_e1']= a[3]
			        lj['left_w0']= a[4]
			        lj['left_w1']= a[5]
			        lj['left_w2']= a[6]
			        print "MOVING"
				left.set_joint_positions(lj)
			      #  left.move_to_joint_positions(lj, timeout=20, threshold = 0.1)


        # Save some CPU time, circa 100Hz publishing.
	r.sleep()

def checkJoints(angles):
	if -2.461 <= angles[0] <= 0.890 and -2.147 <= angles[1] <= 1.047 and -3.028 <= angles[2] <= 3.028 and -0.052 <= angles[3] <= 2.618 and -3.059 <= angles[4] <= 3.059 and -1.571 <= angles[5] <= 2.094 and -3.059 <= angles[6] <= 3.059:
		return True
	return False

if __name__ == '__main__':
    try:
        sender()
    except rospy.ROSInterruptException:
        pass
