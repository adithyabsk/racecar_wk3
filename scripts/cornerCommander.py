#!/usr/bin/env python
"""
commander(git).py

MIT RACECAR 2016

This class will act as a commanding node that reads
(processed) message data from accompanying nodes, makes
decisions on how to proceed, then publishes drive commands
directly to the /navigation topic.

"""


# IMPORTS

import rospy
import math
import time

import numpy as np

from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

from racecar_wk3.msg import BlobDetections
from racecar_wk3.msg import ObjectDetections


# CLASS DECLARATION

class Commander:

    def __init__(self):
        self.DrivePub = rospy.Publisher('/vesc/ackermann_cmd_mux/input/navigation', AckermannDriveStamped,queue_size=10)
        # Add any other topic variables here

        self.SPEED = 0.5
        # Add any other class constants here

        # Add any other class variables here
   
        self.THETA_KP = .9
        self.THETA_KI = .3
        self.THETA_KD = .02
        self.prev_theta = 0
        self.theta_prev_error = 0
        self.theta_prev_time = time.clock()

        self.OBJ_KP = 3
        self.OBJ_KI = 0
        self.OBJ_KD = 0
        self.obj_prev_error = 0
        self.obj_prev_time = time.clock()


    # Function: drive
    # Parameters: speed (float), angle (float)
    #
    # This function will publish a drive command to
    # the /navigation topic in ROS

    def drive(self, angle):
        msg = AckermannDriveStamped()           # Initializes msg variable
        msg.drive.speed = self.SPEED            # Sets msg speed to entered speed
        msg.drive.acceleration = 0              # Sets msg acceleration to 0
        msg.drive.jerk = 1                      # Sets msg jerk to 1
        msg.drive.steering_angle = angle        # Sets msg steering angle to entered angle
        msg.drive.steering_angle_velocity = 1   # Sets msg angle velocity to 1
        self.DrivePub.publish(msg)              # Publishes the message

    def cornerCutter(self, msg):
        #find largest degree and converts to steering angle
        ranges = np.array(msg.ranges) 
        maxPos = np.argmax(ranges) 
        desiredDeg = maxPos/4 - 45

        #new error form where your setpoint can vary (desiredDeg) (prev_theta is you previous theta value)
        error = desiredDeg - prev_theta

        #threshold for zero steering when close enough

        if abs(error) > THRESHOLD: 
            # PUBLISH DRIVE COMMAND
            self.drive(self.calc_pid(self.THETA_KP, self.THETA_KI, self.THETA_KD, error, self.theta_prev_error, self.theta_prev_time))    # Execute drive function
        else:
            self.drive(0)

        self.prev_theta = desiredDeg
        self.theta_prev_error = error
        self.theta_prev_time = time.clock()

    def calc_pid(self, KP, KD, KI, error, prev_error, prev_time):
        e_deriv = (error - prev_error) / (time.clock() - prev_time)
        e_int = (error + prev_error) / 2 * (time.clock() - prev_time)
        return KP*error + KD*e_deriv + KI*e_int
