#!/usr/bin/env python
"""
Path Planning Script for Lab 5
Author: Tiffany Cappellari
"""
import sys

from baxter_interface import Limb

import rospy
import numpy as np
import traceback

from moveit_msgs.msg import OrientationConstraint
from geometry_msgs.msg import PoseStamped

from path_planner import PathPlanner

# Uncomment this line for part 5 of Lab 5
from controller import Controller
from baxter_interface import Limb

import moveit_commander
from moveit_msgs.msg import RobotState

from sensor_msgs.msg import JointState
from std_msgs.msg import Header

def main():
  """
  Main Script
  """

  pub = rospy.Publisher('joint_states', JointState, queue_size=10)
  rate = rospy.Rate(10) # 10hz
  while not rospy.is_shutdown():
  	  joint_state = JointState()
	  joint_state.header = Header()
	  joint_state.header.stamp = rospy.Time.now()
	  joint_state.name = ['base_rotate', 'joint1', 'joint2']
	  joint_state.position = [3.14, 0, 0]
	  pub.publish(joint_state)
	  rate.sleep()

if __name__ == '__main__':
  rospy.init_node('moveit_pub_node')
  main()