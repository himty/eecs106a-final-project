#!/usr/bin/env python
import sys

import rospy
import numpy as np
import traceback

from moveit_msgs.msg import OrientationConstraint
from geometry_msgs.msg import PoseStamped

from path_planner import PathPlanner

import moveit_commander
from moveit_msgs.msg import RobotState

from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import tf

from lab3_skeleton import *

class KinematicsCalculator():
  """
  An instance of this has callable functions that compute
  inverse kinematics and forward kinematics.

  Requirements for forward kinematics:
    - None. Just call the function

  Requirements for inverse kinematics:
    - Run "roscore"
    - Have "roslaunch path_planning arm_bot_publish_moveit.launch" running
  """
  def __init__(self, group_name):
    # Make sure these are the same as the URDF
    # This copy is for faster fk
    self.base_height = 7.5
    self.l1 = 10
    self.l2 = 12

    # Initialize robot params
    self.xis, self.g0 = self.get_joint_twists()

    self.planner = PathPlanner(group_name)
    rospy.sleep(0.5)

  def inverse_kinematics(self, pos, or_quaternion):
    """
    Calls the planner to get joint gives given a position.
    That unfortunately returns a trajectory of many points,
    so we take the last point

    pos- [x, y, z] in cm
    or_quaternion- [or_x, or_y, or_z, or_w]. Target orientation in
                  quaternions

    Returns joint angles [theta1, theta2, theta3, theta4] in radians
    """
    orien_const=[] # idk what this is, but it was in path_test.py
    goal = PoseStamped()
    goal.header.frame_id = "base1"

    #x, y, and z position
    goal.pose.position.x = pos[0]
    goal.pose.position.y = pos[1]
    goal.pose.position.z = pos[2]

    #Orientation as a quaternion
    goal.pose.orientation.x = or_quaternion[0]
    goal.pose.orientation.y = or_quaternion[0]
    goal.pose.orientation.z = or_quaternion[0]
    goal.pose.orientation.w = or_quaternion[0]

    plan = self.planner.plan_to_pose(goal, orien_const)

    trajectory_pts = plan.joint_trajectory.points
    last_pt = trajectory_pts[len(trajectory_pts)-1].positions

    return last_pt

  def get_joint_twists(self):
    q = np.ndarray((3,3))
    w = np.ndarray((3,3))

    q[0:3,0] = [0, 0, 0]
    q[0:3,1] = [0, 0, self.base_height]
    q[0:3,2] = [0, self.l1, self.base_height]

    w[0:3,0] = [0, 0, 1]
    w[0:3,1] = [1, 0, 0]
    w[0:3,2] = [1, 0, 0]

    R = np.eye(3) # Initially the same as the base frame
    q_init = np.array([0, self.l1+self.l2, self.base_height])
    g0 = get_3dto4d(R, q_init)

    # write the twist xi_i for each joint in the manipulator
    xis = np.ndarray((6, 3))
    for i in range(3):
      xis[:,i] = get_xi(w[:,i], q[:,i])

    return xis, g0

  def forward_kinematics(self, joint_angles):
    """
    joint_angles- [theta1, theta2, theta3, theta4] in radians
                for base_rotate, joint1, joint2, and joint3
    returns (position, angle)
      where position is [x, y, z] in spatial coordinates. in cm
            and angle is in euler angles [roll, pitch, yaw]
    """
    g = prod_exp(self.xis, np.array(joint_angles))
    g_ee = g.dot(self.g0) # pose of end effector

    quaternion = tf.transformations.quaternion_from_matrix(g_ee)
    euler = tf.transformations.euler_from_quaternion(quaternion)

    pos = g_ee[:3,3]
    return pos, euler

def main():
  """
  Maintf.transformations.quaternion_from_matrix(g_ee)
 Script
  """
  #group = moveit_commander.MoveGroupCommander('arm')


  kinematics = KinematicsCalculator("arm")
  import time
  
  starttime = time.time()
  pos, rotation = kinematics.forward_kinematics([1, 1, 1])
  print('fk result', pos, rotation)
  print('fk time', time.time() - starttime)

  starttime = time.time()
  print('ik result', kinematics.inverse_kinematics(pos, rotation))
  print('ik time', time.time() - starttime)

if __name__ == '__main__':
  rospy.init_node('moveit_node')
  main()