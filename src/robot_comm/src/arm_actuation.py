#!/usr/bin/env python

"""
ROS network for path planning and output
"""

import rospy
import sys
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
sys.path.insert(0,'/home/jon/ros_workspaces/eecs106a-final-project/src/path_planning/moveit_planner')
sys.path.insert(0,'/home/jon/ros_workspaces/eecs106a-final-project/src/path_planning/moveit_planner')

print(sys.path)

from robot_comm_msg.msg import AngleArr

from stamped_command_spheres_msg.msg import StampedCommandSpheres
from command_spheres_msg.msg import CommandSpheres

from kinematics_calculator_moveit import KinematicsCalculator

from next_pt_planner import NextPointPlanner

#import user_input.vision as cv

import numpy as np

joint_states = [0, 0, 0]

curr_sphere_pos = np.array([0, -3, 0, 1])
curr_sphere_cmd = "near"

def updateJoints(data):
    global joint_states
    joint_states = data.position

def updateSpheres(data):
    global curr_sphere_pos, curr_sphere_cmd
    #can support multiple spheres 
    curr_sphere = data.spheres[0]
    curr_sphere_pos[0] = curr_sphere.x
    curr_sphere_pos[1] = curr_sphere.y
    curr_sphere_pos[2] = curr_sphere.z
    #currently color based: curr_sphere_cmd = curr_sphere.cmd_name
    #hardcode:
    curr_sphere_cmd = "near"

def publish_joint_angles(pub, joint_angles):
    joint_state = JointState()
    joint_state.header = Header()
    joint_state.header.stamp = rospy.Time.now()
    joint_state.name = ['base_rotate', 'joint1', 'joint2']
    joint_state.position = np.array(joint_angles)*np.pi/180
    pub.publish(joint_state)

def cmd_angle():

    #callibration sequence launched from CV node

    rospy.Subscriber("joint_states", JointState, updateJoints)
    joint_pub = rospy.Publisher("joint_states", JointState, queue_size=10)
    publish_joint_angles(joint_pub, [0, 0, 0])

    rospy.Subscriber("vision_spheres", StampedCommandSpheres, updateSpheres)

    pub = rospy.Publisher('cmd_angle', AngleArr, queue_size=10)
    
    r = rospy.Rate(10) # 10hz

    arm = KinematicsCalculator('arm')

    next_pt_planner = NextPointPlanner(max_dist_per_timestep=2)

    while not rospy.is_shutdown():

        raw_input('Press enter to actuate arm:')

        try:

            g = arm.forward_kinematics(joint_states)

            arm_pos = g[:3, 3]

            #convert back to spatial g*coordinates
            sphere_coords_spatial = np.matmul(g, curr_sphere_pos)[:3]
            
            if (curr_sphere_cmd == "near"):
                target_coords_spatial = next_pt_planner.get_near_point(arm_pos, sphere_coords_spatial)
            else:
                target_coords_spatial = next_pt_planner.get_far_point(arm_pos, sphere_coords_spatial)
            
            print("target_coords_spatial", target_coords_spatial)

            angles = arm.inverse_kinematics(target_coords_spatial)

            publish_joint_angles(joint_pub, angles)
            print(angles)

            angles = angles.astype(np.int16).tolist()
            print(angles)

            pub_string = AngleArr(angles, rospy.get_time())
            pub.publish(pub_string)
            r.sleep()
            
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

#Python's syntax for a main() method
if __name__ == '__main__':

    rospy.init_node('cmd_angle', anonymous=True)

    try:
        cmd_angle()

    except rospy.ROSInterruptException: pass