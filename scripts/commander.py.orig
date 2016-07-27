#!/usr/bin/env python
"""
particleCommander(git).py

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

class ParticleCommander:

    def __init__(self):
<<<<<<< HEAD
<<<<<<< HEAD
        #self.DrivePub = rospy.Publisher('/vesc/ackermann_cmd_mux/input/navigation', AckermannDriveStamped,queue_size=10)
        self.DrivePub = rospy.Publisher('/racecar/ackermann_cmd_mux/input/navigation', AckermannDriveStamped,queue_size=10) #gazebo pulisher
=======
        self.DrivePub = rospy.Publisher('/vesc/ackermann_cmd_mux/input/navigation', AckermannDriveStamped,queue_size=10)
        #self.DrivePub = rospy.Publisher('/racecar/ackermann_cmd_mux/input/navigation', AckermannDriveStamped,queue_size=10) #gazebo pulisher
>>>>>>> 8175b2f1f2d12e0ab8c768ec893c8c01d5707114
        # Add any other topic variales here

        #self.SPEED = 0.5

=======
        self.DrivePub = rospy.Publisher('/vesc/ackermann_cmd_mux/input/navigation', AckermannDriveStamped,queue_size=0)
        # Add any other topic variables here

        self.SPEED = 2
>>>>>>> c5376eaed2002d0f88e8f4fbbbaf4096089c29eb
        # Add any other class constants here

        self.PUSH_VECTOR = np.array([0, 1])
        self.FORCE_CONSTANT = 0.1 # the numerator in the F = (kQq)/r^2
        self.HOKUYO_ANGLES = np.arange(-45,225.25,0.25) #change back to np.arange(-45,225.25,0.25)
        self.p_dir = 1.0
        self.p_mag = 0.03
        #print len(self.HOKUYO_ANGLES)
        # Add any other class variables here
<<<<<<< HEAD
=======
   
        self.WALL_KP = .9
        self.WALL_KI = .3
        self.WALL_KD = .02
        self.wall_prev_error = 0
        self.wall_prev_time = time.clock()
        self.WALL_DDES = 0.4
        self.wall_right = False  # which wall to follow

        self.OBJ_KP = .01
        self.OBJ_KI = .00
        self.OBJ_KD = 0
        self.obj_prev_error = 0
        self.obj_prev_time = time.clock()

>>>>>>> c5376eaed2002d0f88e8f4fbbbaf4096089c29eb

    # Function: drive
    # Parameters: speed (float), angle (float)
    #
    # This function will publish a drive command to
    # the /navigation topic in ROS

    def drive(self, direction, magnitude):
        msg = AckermannDriveStamped()           # Initializes msg variable
        msg.drive.speed = magnitude             # Sets msg speed to entered speed
        #msg.drive.acceleration = 1              # Sets msg acceleration to 0
        #msg.drive.jerk = 1                      # Sets msg jerk to 1
        msg.drive.steering_angle = direction    # Sets msg steering angle to entered angle
        #msg.drive.steering_angle_velocity = 1   # Sets msg angle velocity to 1
        self.DrivePub.publish(msg)              # Publishes the message
<<<<<<< HEAD
        
    def avoidObjects(self, msg):
        #parameters speed should go from -1 to 1
        #Calculate Force Vector
        ranges = np.array(msg.ranges) #Only range values at this point
        forces = self.FORCE_CONSTANT / np.power(ranges, 2)
        xComp = np.multiply(forces, np.cos(np.deg2rad(self.HOKUYO_ANGLES)))
        yComp = np.multiply(forces, np.sin(np.deg2rad(self.HOKUYO_ANGLES)))
        #fVec = np.column_stack((xComp, yComp))
        finalFVec = np.negative([np.sum(xComp), np.sum(yComp)])

        #Calculate Resultant Force
        rsltFVec = np.add(finalFVec, self.PUSH_VECTOR)

        #Convert rectangular to polar
        direction = self.p_dir*math.atan2(rsltFVec[1],rsltFVec[0])#*np.sign(rsltFVec[0])
        print direction
        magnitude = self.p_mag*np.sqrt(rsltFVec[0]**2+rsltFVec[1]**2)
        print magnitude
        
        self.drive(direction, magnitude)
<<<<<<< HEAD
=======

    def evade_objects(self, msg):
        #msg.dists += (.01,)  # fake left wall
        #msg.lefts += (140,)
        #msg.rights += (270,)
        objs = [d for d in msg.dists if d < 3]  # dists in order of distance
        objs.sort()
	if len(objs) >= 2:
            thresh_dist = objs[1]
        elif objs:
            thresh_dist = objs[len(objs)-1]
        else:
            self.drive(0)
            return
        close_objs = [i for i in range(len(msg.dists)) if msg.dists[i]-thresh_dist < .2]  # consider objects that are 1 meter within closest object
        max_space = -1
        max_center = None
        for s in range(len(close_objs)-1):
            space = msg.lefts[close_objs[s+1]] - msg.rights[close_objs[s]]
            if space > max_space:
                max_space = space
                max_center = (msg.lefts[close_objs[s+1]] + msg.rights[close_objs[s]]) / 2
        if max_center is None:
            rospy.loginfo("no space, we're doomed")
        else:
            error = max_center - 135
            if abs(error) > 3: 
                # PUBLISH DRIVE COMMAND
                self.drive(self.calc_pid(self.OBJ_KP, self.OBJ_KD, self.OBJ_KI, error, self.obj_prev_error, self.obj_prev_time))    # Execute drive function
            else:
                self.drive(0)
            self.obj_prev_error = error
        self.obj_prev_time = time.clock()


    # Function: wall_follow
    # Parameters: msg (ObjectDetection)
    #
    # This function uses a PID control system
    # to let the car follow a line

    def wall_follow(self, msg):

        if self.wall_right:
            start_angle = 20
            end_angle = 125
            mult = 1
        else:
            start_angle = 145
            end_angle = 250
            mult = -1
        
        try:
            min_ind = min([i for i in range(len(msg.dists)) if (start_angle < msg.lefts[i] and msg.lefts[i] < end_angle) or (start_angle < msg.rights[i] and msg.rights[i] < end_angle) or (msg.lefts[i] < start_angle and msg.rights[i] > end_angle)], key=lambda x: msg.dists[x])
        except ValueError:  # no wall detected
            rospy.loginfo("no wall detected")
            self.drive(0)
            return

        dist = msg.dists[min_ind]       # Finds the minimum range

        error = self.WALL_DDES - dist
        
        # SET PID PARAMETERS
        THRESHOLD = 0.05                # Sets threshold to 5cm
        
        if abs(error) > THRESHOLD: 
            # PUBLISH DRIVE COMMAND
            self.drive(mult * self.calc_pid(self.WALL_KP, self.WALL_KD, self.WALL_KI, error, self.wall_prev_error, self.wall_prev_time))    # Execute drive function
        else:
            self.drive(0)
        
        self.wall_prev_error = error
        self.wall_prev_time = time.clock()

    def calc_pid(self, KP, KD, KI, error, prev_error, prev_time):
        e_deriv = (error - prev_error) / (time.clock() - prev_time)
        e_int = (error + prev_error) / 2 * (time.clock() - prev_time)
        return KP*error + KD*e_deriv + KI*e_int
>>>>>>> c5376eaed2002d0f88e8f4fbbbaf4096089c29eb
=======
>>>>>>> 8175b2f1f2d12e0ab8c768ec893c8c01d5707114
