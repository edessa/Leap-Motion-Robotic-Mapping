################################################################################
# Copyright (C) 2012-2013 Leap Motion, Inc. All rights reserved.               #
# Leap Motion proprietary and confidential. Not for distribution.              #
# Use subject to the terms of the Leap Motion SDK Agreement available at       #
# https://developer.leapmotion.com/sdk_agreement, or another agreement         #
# between Leap Motion and you, your company or other organization.             #
################################################################################

import Leap, sys, thread, time
from Leap import CircleGesture, KeyTapGesture, ScreenTapGesture, SwipeGesture
import argparse
import numpy as np
import rospy
import math
import array
from array import *
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import numpy
count = 0
class SampleListener(Leap.Listener):
    pub = rospy.Publisher('allegroHand_0/joint_cmd', JointState, queue_size=10)
    rospy.init_node('joint_state_publisher')
    rate = rospy.Rate(100)
    finger_names = ['Thumb', 'Index', 'Middle', 'Ring', 'Pinky']
    bone_names = ['Metacarpal', 'Proximal', 'Intermediate', 'Distal']
    state_names = ['STATE_INVALID', 'STATE_START', 'STATE_UPDATE', 'STATE_END']
    print "HI"
    measurements = numpy.zeros(16)
    def on_init(self, controller):
        print "Initialized"

    def on_disconnect(self, controller):
        # Note: not dispatched when running in a debugger.
        print "Disconnected"

    def on_exit(self, controller):
        print "Exited"

    def on_frame(self, controller):
	global count
	count += 1
        # Get the most recent frame and report some basic information
        frame = controller.frame()
        hello = JointState()
    	hello.header = Header()
        print "Frame id: %d, timestamp: %d, hands: %d, fingers: %d, tools: %d, gestures: %d" % (
              frame.id, frame.timestamp, len(frame.hands), len(frame.fingers), len(frame.tools), len(frame.gestures()))

	anglesOpen = numpy.array([0, 2.902, 0.2797, 0.218456, 0, 2.7, 0.223, 0.2389, 0, 2.702, 0.2969, 0.2654, 0, 0, 0, 0.024187])
	anglesClosed = numpy.array([0, 3, 1.3189, 0.6688, 0, 3, 1.3845, 0.6785, 0, 3.0319, 1.4279, 0.66959, 0, 0, 0, 1.8233])

	anglesClosedAllegro = numpy.array([0.3052, 1.6, 1.5, 1.5, 0.3, 1.7, 1.4, 1.4, 0.35, 1.6, 1.4360, 1.453, 0.4488, 1.1249, 0.0882, -0.03005])
	anglesOpenAllegro = numpy.zeros(16)
	anglesOpenAllegro += [.3] * 16
#joints 15-12: thumb, joints 11-8: index, joints 7-4: middle, joints 3-1: ring, ignore 0, 4, 8, 13, 12

	hello.header.stamp = rospy.Time.now()
	hello.name = ['joint_0.0', 'joint_1.0', 'joint_2.0', 'joint_3.0', 'joint_4.0', 'joint_5.0', 'joint_6.0', 'joint_7.0', 'joint_8.0', 'joint_9.0', 'joint_10.0', 'joint_11.0', 'joint_12.0', 'joint_13.0', 'joint_14.0', 'joint_15.0']
	hello.velocity = []
	hello.effort = []
	measurements2 = numpy.zeros((5,3))

        # Get hands
        for hand in frame.hands:

            handType = "Left hand" if hand.is_left else "Right hand"

            # Get the hand's normal vector and direction
            normal = hand.palm_normal
            direction = hand.direction
	    i = 0

            # Get fingers
            for finger in hand.fingers:
            	direction2 = hand.arm.direction
	    	angle1 = direction2.angle_to(finger.bone(0).direction)
	    	angle2 = finger.bone(0).direction.angle_to(finger.bone(1).direction)
	    	angle3 = finger.bone(1).direction.angle_to(finger.bone(3).direction)
		measurements2[i] = [angle1, angle2, angle3]
		i += 1
	  
	self.measurements[1] += measurements2[1][0]
	self.measurements[2] += measurements2[1][1]
	self.measurements[3] += measurements2[1][2]
	self.measurements[5] += measurements2[2][0]
	self.measurements[6] += measurements2[2][1]
	self.measurements[7] += measurements2[2][2]
	self.measurements[9] += measurements2[3][0]
	self.measurements[10] += measurements2[3][1]
	self.measurements[11] += measurements2[3][2]
	self.measurements[15] += measurements2[4][2]
	

	if count % 10 == 0:
		self.measurements = self.measurements / 10.0
		distanceLeap = anglesClosed - anglesOpen
		distanceAllegro = anglesClosedAllegro - anglesOpenAllegro
		accurateDistance = self.measurements - anglesOpen
		alpha = accurateDistance / distanceLeap
	
		setValues = alpha * distanceAllegro + anglesOpenAllegro #joints 15-12: thumb, joints 11-8: index, joints 7-4: middle, joints 3-1: ring, ignore 0, 4, 8, 13, 12
		setValues[0] = 0
		setValues[4] = 0
		setValues[8] = 0
		setValues[13] = 0
		setValues[12] = 0
		setValues[14] = 0
		print setValues
		hello.position = setValues
		self.pub.publish(hello)
		self.measurements = numpy.zeros(16)
	print count
	self.rate.sleep()


    def state_string(self, state):
        if state == Leap.Gesture.STATE_START:
            return "STATE_START"

        if state == Leap.Gesture.STATE_UPDATE:
            return "STATE_UPDATE"

        if state == Leap.Gesture.STATE_STOP:
            return "STATE_STOP"

        if state == Leap.Gesture.STATE_INVALID:
            return "STATE_INVALID"

def main():
    # Create a sample listener and controller
    listener = SampleListener()
    controller = Leap.Controller()

    # Have the sample listener receive events from the controller
    controller.add_listener(listener)

    # Keep this process running until Enter is pressed
    print "Press Enter to quit..."
    try:
        sys.stdin.readline()
    except KeyboardInterrupt:
        pass
    finally:
        # Remove the sample listener when done
        controller.remove_listener(listener)


if __name__ == "__main__":
    main()
