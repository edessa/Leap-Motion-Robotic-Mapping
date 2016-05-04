#################################################################################
# Copyright (C) 2012-2013 Leap Motion, Inc. All rights reserved.                #
# Leap Motion proprietary and confidential. Not for distribution.               #
# Use subject to the terms of the Leap Motion SDK Agreement available at        #
# https://developer.leapmotion.com/sdk_agreement, or another agreement          #
# between Leap Motion and you, your company or other organization.              #
#################################################################################

#################################################################################
# Altered LEAP example by Florian Lier, you need to have the LEAP SDK installed #
# for this to work properly ;)                                                  #
# This interface provides access to the LEAP MOTION hardware, you will need to  #
# have the official LEAP MOTION SDK installed in order to load the shared       #
# provided with the SDK.                                                        #
#################################################################################

import sys
import time
# Set (append) your PYTHONPATH properly, or just fill in the location of your LEAP
# SDK folder, e.g., $HOME/LeapSDK/lib where the Leap.py lives and /LeapSDK/lib/x64 or
# x86 where the *.so files reside.

# Below, you can see the "dirty" version - NOT RECOMMENDED!

sys.path.append("/home/edessale/Desktop/LeapDeveloperKit_2.3.1+31549_linux/LeapSDK/lib")
sys.path.append("/home/edessale/Desktop/LeapDeveloperKit_2.3.1+31549_linux/LeapSDK/lib/x64")
import threading
import Leap
from Leap import CircleGesture, KeyTapGesture, ScreenTapGesture, SwipeGesture

class LeapInterface(Leap.Listener):
    def on_init(self, controller):
        # These variables as probably not thread safe
        # TODO: Make thread safe ;)
        self.hand           = [0,0,0]
        self.right_hand = False
        self.left_hand = False
        self.hand_direction = [0,0,0]
        self.hand_normal    = [0,0,0]
        self.hand_palm_pos  = [0,0,0]
        self.hand_pitch     = 0.0
        self.hand_yaw       = 0.0
        self.hand_roll      = 0.0
	
        self.hand2           = [0,0,0]
        self.hand_direction2 = [0,0,0]
        self.hand_normal2    = [0,0,0]
        self.hand_palm_pos2  = [0,0,0]
        self.hand_pitch2     = 0.0
        self.hand_yaw2       = 0.0
        self.hand_roll2      = 0.0
        print "Initialized Leap Motion Device"

    def on_connect(self, controller):
        print "Connected to Leap Motion Controller"

        # Enable gestures
        controller.enable_gesture(Leap.Gesture.TYPE_CIRCLE);
        controller.enable_gesture(Leap.Gesture.TYPE_KEY_TAP);
        controller.enable_gesture(Leap.Gesture.TYPE_SCREEN_TAP);
        controller.enable_gesture(Leap.Gesture.TYPE_SWIPE);

    def on_disconnect(self, controller):
        # Note: not dispatched when running in a debugger.
        print "Disconnected Leap Motion"

    def on_exit(self, controller):
        print "Exited Leap Motion Controller"

    def on_frame(self, controller):
        # Get the most recent frame and report some basic information
        frame = controller.frame()

    #    print "Frame id: %d, timestamp: %d, hands: %d, fingers: %d, tools: %d, gestures: %d" % (
     #         frame.id, frame.timestamp, len(frame.hands), len(frame.fingers), len(frame.tools), len(frame.gestures()))

        if not frame.hands.is_empty: #recently changed in API
            # Get the first hand
            
            
            #we are seeking one left and one right hands
            there_is_right_hand=False
            there_is_left_hand=False
            i = 0
            for hand in frame.hands:            
                if hand.is_right:
                    there_is_right_hand=True
                    self.right_hand=hand
		    if len(frame.hands) == 2:
		        self.hand = hand
		    i = 1
                elif hand.is_left:
                    there_is_left_hand=True      
                    self.left_hand=hand
		    if len(frame.hands) == 2:
		        self.hand2 = hand
                    i = 0
            if not there_is_right_hand:
                self.right_hand=False
            
            if not there_is_left_hand:
                self.left_hand=False
                       

	    if len(frame.hands) == 1:
		self.hand = frame.hands[0]
	    
            # Check if the hand has any fingers
            #fingers = self.hand.fingers
            #if not fingers.empty:
                # Calculate the hand's average finger tip position
                #avg_pos = Leap.Vector()
                #for finger in fingers:
                    #avg_pos += finger.tip_position
                # avg_pos /= len(fingers)
                # print "Hand has %d fingers, average finger tip position: %s" % (len(fingers), avg_pos)

            # Get the hand's sphere radius and palm position
            # print "Hand sphere radius: %f mm, palm position: %s" % (self.hand.sphere_radius, hand.palm_position)

            # Get the hand's normal vector and direction
            normal = self.hand.palm_normal
            direction = self.hand.direction
            pos = self.hand.palm_position

	    if len(frame.hands) == 2:
	        normal2 = self.hand2.palm_normal
                direction2 = self.hand2.direction
                pos2 = self.hand2.palm_position

            self.hand_direction[0] = direction.x
            self.hand_direction[1] = direction.y
            self.hand_direction[2] = direction.z
            self.hand_normal[0]    = normal.x
            self.hand_normal[1]    = normal.y
            self.hand_normal[2]    = normal.z
            self.hand_palm_pos[0]  = pos.x
            self.hand_palm_pos[1]  = pos.y
            self.hand_palm_pos[2]  = pos.z
            self.hand_pitch        = direction.pitch
            self.hand_yaw          = normal.yaw
            self.hand_roll         = direction.roll

	    if len(frame.hands) == 2:
                self.hand_direction2[0] = direction2.x
                self.hand_direction2[1] = direction2.y
                self.hand_direction2[2] = direction2.z
                self.hand_normal2[0]    = normal2.x
                self.hand_normal2[1]    = normal2.y
                self.hand_normal2[2]    = normal2.z
                self.hand_palm_pos2[0]  = pos2.x
                self.hand_palm_pos2[1]  = pos2.y
                self.hand_palm_pos2[2]  = pos2.z
                self.hand_pitch2        = direction2.pitch
                self.hand_yaw2          = normal2.yaw
                self.hand_roll2         = direction2.roll


            # Calculate the hand's pitch, roll, and yaw angles
            #print "Hand pitch: %f degrees, roll: %f degrees, yaw: %f degrees" % (self.hand_pitch, self.hand_roll, self.hand_yaw)

            '''
            # Gestures
            for gesture in frame.gestures():
                if gesture.type == Leap.Gesture.TYPE_CIRCLE:
                    circle = CircleGesture(gesture)

                    # Determine clock direction using the angle between the pointable and the circle normal
                    if circle.pointable.direction.angle_to(circle.normal) <= Leap.PI/4:
                        clockwiseness = "clockwise"
                    else:
                        clockwiseness = "counterclockwise"

                    # Calculate the angle swept since the last frame
                    swept_angle = 0
                    if circle.state != Leap.Gesture.STATE_START:
                        previous_update = CircleGesture(controller.frame(1).gesture(circle.id))
                        swept_angle =  (circle.progress - previous_update.progress) * 2 * Leap.PI

                    print "Circle id: %d, %s, progress: %f, radius: %f, angle: %f degrees, %s" % (
                            gesture.id, self.state_string(gesture.state),
                            circle.progress, circle.radius, swept_angle * Leap.RAD_TO_DEG, clockwiseness)

                if gesture.type == Leap.Gesture.TYPE_SWIPE:
                    swipe = SwipeGesture(gesture)
                    print "Swipe id: %d, state: %s, position: %s, direction: %s, speed: %f" % (
                            gesture.id, self.state_string(gesture.state),
                            swipe.position, swipe.direction, swipe.speed)

                if gesture.type == Leap.Gesture.TYPE_KEY_TAP:
                    keytap = KeyTapGesture(gesture)
                    print "Key Tap id: %d, %s, position: %s, direction: %s" % (
                            gesture.id, self.state_string(gesture.state),
                            keytap.position, keytap.direction )

                if gesture.type == Leap.Gesture.TYPE_SCREEN_TAP:
                    screentap = ScreenTapGesture(gesture)
                    print "Screen Tap id: %d, %s, position: %s, direction: %s" % (
                            gesture.id, self.state_string(gesture.state),
                            screentap.position, screentap.direction )

        if not (frame.hands.empty and frame.gestures().empty):
            print ""

    def state_string(self, state):
        if state == Leap.Gesture.STATE_START:
            return "STATE_START"

        if state == Leap.Gesture.STATE_UPDATE:
            return "STATE_UPDATE"

        if state == Leap.Gesture.STATE_STOP:
            return "STATE_STOP"

        if state == Leap.Gesture.STATE_INVALID:
            return "STATE_INVALID"
    '''

    def get_hand_direction(self):
        return self.hand_direction

    def get_hand_normal(self):
        return self.hand_normal

    def get_hand_palmpos(self):
        return self.hand_palm_pos

    def get_hand_yaw(self):
        return self.hand_yaw

    def get_hand_pitch(self):
        return self.hand_pitch

    def get_hand_roll(self):
        return self.hand_roll


    def get_hand_direction2(self):
        return self.hand_direction2

    def get_hand_normal2(self):
        return self.hand_normal2

    def get_hand_palmpos2(self):
        return self.hand_palm_pos2

    def get_hand_yaw2(self):
        return self.hand_yaw2

    def get_hand_pitch2(self):
        return self.hand_pitch2

    def get_hand_roll2(self):
        return self.hand_roll2

    def getNumHands(self):
	if self.right_hand and self.left_hand:
	    return 2
	if self.right_hand:
	    return 1
	if self.left_hand:
	    return 0
	return -1



class Runner(threading.Thread):

    def __init__(self,arg=None):
        threading.Thread.__init__(self)
        self.arg=arg
        self.listener = LeapInterface()
        self.controller = Leap.Controller()
        self.controller.add_listener(self.listener)
    
    def __del__(self):
        self.controller.remove_listener(self.listener)

    def get_hand_direction(self):
        return self.listener.get_hand_direction()

    def get_hand_normal(self):
        return self.listener.get_hand_normal()

    def get_hand_palmpos(self):
        return self.listener.get_hand_palmpos()

    def get_hand_roll(self):
        return self.listener.get_hand_roll()

    def get_hand_pitch(self):
        return self.listener.get_hand_pitch()

    def get_hand_yaw(self):
        return self.listener.get_hand_yaw()

    def getNumHands(self):
        return self.listener.getNumHands()

    def get_hand_direction2(self):
        return self.listener.get_hand_direction2()

    def get_hand_normal2(self):
        return self.listener.get_hand_normal2()

    def get_hand_palmpos2(self):
        return self.listener.get_hand_palmpos2()

    def get_hand_roll2(self):
        return self.listener.get_hand_roll2()

    def get_hand_pitch2(self):
        return self.listener.get_hand_pitch2()

    def get_hand_yaw2(self):
        return self.listener.get_hand_yaw2()


    def run (self):
        while True:
            # Save some CPU time
            time.sleep(0.001)

