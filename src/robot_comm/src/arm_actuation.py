#!/usr/bin/env python

"""
ROS network for path planning and output
"""

import rospy
import sys
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
import time

sys.path.insert(0,'/home/jon/ros_workspaces/eecs106a-final-project/src/path_planning/moveit_planner')
sys.path.insert(0,'/home/jon/ros_workspaces/eecs106a-final-project/src/path_planning/moveit_planner')

from robot_comm_msg.msg import AngleArr
from stamped_command_spheres_msg.msg import StampedCommandSpheres
from command_sphere_msg.msg import CommandSphere
from kinematics_calculator_moveit import KinematicsCalculator
from next_pt_planner import NextPointPlanner
import numpy as np
import cv2


from test_aruco import ArucoDetector

joint_states = np.array([-90, 90, -90])

seen_sphere = False

curr_sphere_pos = np.array([0, 0, 0, 1])
curr_sphere_cmd = "near"

detect_mode = "aruco" # aruco or sphere

def updateSpheres(data):
    global curr_sphere_pos, curr_sphere_cmd, seen_sphere
    #can support multiple spheres 
    if len(data.spheres) == 0:
        return

    curr_sphere = data.spheres[0]
    curr_sphere_pos[0] = curr_sphere.x
    curr_sphere_pos[1] = curr_sphere.y
    curr_sphere_pos[2] = curr_sphere.z

    #currently color based: curr_sphere_cmd = curr_sphere.cmd_name
    #hardcode:
    #curr_sphere_cmd = "near"
    seen_sphere = True

def publish_joint_states(joint_pub, arm_pub):
    global joint_states

    joint_state = JointState()
    joint_state.header = Header()
    joint_state.header.stamp = rospy.Time.now()
    joint_state.name = ['base_rotate', 'joint1', 'joint2']
    joint_state.position = joint_states*np.pi/180
    joint_pub.publish(joint_state)

    angles = joint_states.astype(np.int16).tolist()
    angles[0] = -angles[0]
    angles[2] = angles[2] + 90
    arm_pub.publish(AngleArr(angles, rospy.get_time()))

def dumb_ik():
    global joint_states

    x = curr_sphere_pos[0]
    y = curr_sphere_pos[1]
    z = curr_sphere_pos[2]

    if x > 0:
        joint_states[0] -= 2
    elif x < 0:
        joint_states[0] += 2

    if y > 0:
        joint_states[1] += 2
    elif y < 0:
        joint_states[1] -= 2

    if z > 15:
        joint_states[1] -= .5
        joint_states[2] += .5
    else:
        joint_states[1] += .5
        joint_states[2] -= .5


    joint_states[0] = max(-90, min(90, joint_states[0]))
    joint_states[1] = max(0, min(180, joint_states[1]))
    joint_states[2] = max(-90, min(90, joint_states[2]))

    return np.array(joint_states)

def cmd_angle():
    global seen_sphere, joint_states
    #callibration sequence launched from CV node

    joint_pub = rospy.Publisher("joint_states", JointState, queue_size=10)
    rospy.Subscriber("vision_spheres", StampedCommandSpheres, updateSpheres)
    arm_pub = rospy.Publisher('cmd_angle', AngleArr, queue_size=10)

    r = rospy.Rate(1) # 10hz

    publish_joint_states(joint_pub, arm_pub)
    r.sleep()

    arm = KinematicsCalculator('arm')
    next_pt_planner = NextPointPlanner(max_dist_per_timestep=2)

    if detect_mode == "aruco":
        aruco_detector = ArucoDetector()

    print("hey")

    while not rospy.is_shutdown():
        try:
            if detect_mode == "aruco":
                # Run the detector in this loop for simplicity
                aruco_pos = aruco_detector.get_pos()
                cv2.waitKey(5)
                if aruco_pos is not None:
                    curr_sphere_pos = aruco_pos
                    seen_sphere = True
                else:
                    seen_sphere = False                    

            if not seen_sphere:
                continue

            seen_sphere = False

            print("joint_states")
            print(joint_states)

            g = arm.forward_kinematics(joint_states)

            print("g")
            print(g)

            arm_pos = g[:3, 3]

            print("arm_pos")
            print(arm_pos)

            print("SPHERE CAMERA")
            print(curr_sphere_pos)

            # x = -x
            # y = z
            # z = y

            # x = -curr_sphere_pos[0]
            # y = curr_sphere_pos[2]
            # z = curr_sphere_pos[1]
            
            # FAR BASE
            # x=-7 CCW
            # x=7 CW

            # NEAR Tilt
            # z = 7 UP
            # z = -7 DOWN

            # NEAR Extend
            # y = 7 extend
            # y = -7 retract

            x = -curr_sphere_pos[0] * 7
            y = 0 # curr_sphere_pos[2]
            z = -curr_sphere_pos[1] * 7

            print("SPHERE END EFFECTOR")
            print([x, y, z])

            #convert back to spatial g*coordinates
            sphere_coords_spatial = np.matmul(g, [x, y, z, 1])[:3]

            print("SPHERE SPATIAL")
            print(sphere_coords_spatial)

            if (curr_sphere_cmd == "near"):
                target_coords_spatial = next_pt_planner.get_near_point(arm_pos, sphere_coords_spatial)
            else:
                target_coords_spatial = next_pt_planner.get_far_point(arm_pos, sphere_coords_spatial)
            
            print("target_coords_spatial")
            print(target_coords_spatial)

            if target_coords_spatial is None:
                continue

            new_joint_states = arm.inverse_kinematics(target_coords_spatial)
            #angles = dumb_ik()

            if new_joint_states is None:
                continue

            joint_states = new_joint_states

            publish_joint_states(joint_pub, arm_pub)

            r.sleep()
            
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

#Python's syntax for a main() method
if __name__ == '__main__':

    rospy.init_node('cmd_angle', anonymous=True)

    try:
        cmd_angle()

    except rospy.ROSInterruptException: pass