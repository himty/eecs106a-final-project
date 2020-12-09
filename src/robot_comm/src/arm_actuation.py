#!/usr/bin/env python

"""
ROS network for path planning and output
"""

import rospy
from path_planning.moveit_planner.kinematics_calculator_moveit import KinematicsCalculator

#import user_input.vision as cv

import numpy as np

from robot_comm_msg.msg import AngleArr

joint_states = [0, 0, 0]

target_coords_ef = [0, 0, 0]

def updateJoints(data):
    joint_states = data.position

def updateSpheres(data):
    #update with correct message type
    target_coords_ef = data.position

def cmd_angle():

    #callibration sequence launched from CV node

    rospy.Subscriber("joint_states", String, updateJoints)

    #placeholder topic 'spheres', update later
    rospy.Subscriber("spheres", String, updateSpheres)

    pub = rospy.Publisher('cmd_angle', AngleArr, queue_size=10)
    
    r = rospy.Rate(10) # 10hz

    arm = KinematicsCalculator('arm')

    while not rospy.is_shutdown():

        raw_input('Press enter to actuate arm:')

        try:

            g = arm.forward_kinematics(joint_states)

            #convert back to spatial g*coordinates
            target_coords_spatial = np.matmul(g, target_coords_ef)

            angles = arm.inverse_kinematics(target_coords_spatial) 
            
            print(angles)

            pub_string = AngleArr(angles, rospy.get_time())

        	pub.publish(pub_string)

        	r.sleep()
            
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

#Python's syntax for a main() method
if __name__ == '__main__':

	rospy.init_node('cmd_angle', anonymous=True)
    rospy.init_node('joint_listener', anonymous=True)
    rospy.init_node('cv_listener', anonymous=True)

	try:
	  cmd_angle()

	except rospy.ROSInterruptException: pass