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

class KinematicsCalculator():
  """
  An instance of this has callable functions that compute
  inverse kinematics and forward kinematics.

  Requirements for forward kinematics:
    - Have a node continuously publishing JointState messages to
        'joint_states'
        - For example, path_planning/moveit_planner.moveit_ik_test_pub.py does this

  Requirements for inverse kinematics:
    - Run "roscore"
    - Have "roslaunch path_planning arm_bot_publish_moveit.launch" running
  """

  STATUS_WAIT_PUB = 0
  STATUS_WAIT_CALC = 1
  STATUS_DONE = 2

  MAX_FK_TRIES = 20

  def __init__(self, group_name):
    self.fk_status = KinematicsCalculator.STATUS_DONE
    self.fk_result = (None, None)
    self.group = moveit_commander.MoveGroupCommander(group_name)

    self.joint_state_sub = rospy.Subscriber("joint_states", JointState, self.get_forward_kinematics_cbk())
    
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

  def get_forward_kinematics_cbk(self):
    # to get self in scope
    # Only for use in this class. Gets the pose then sets a variable
    def forward_kinematics_cbk(msg):
      # Only do this if we were told to do forward kinematics
      if self.fk_status == KinematicsCalculator.STATUS_WAIT_PUB:
        self.fk_status = KinematicsCalculator.STATUS_WAIT_CALC

        pose = self.group.get_current_pose().pose
        pos = (pose.position.x, pose.position.y, pose.position.z)
        quaternion = (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.fk_result = (pos, euler)
        self.fk_status = KinematicsCalculator.STATUS_DONE

    return forward_kinematics_cbk

  def forward_kinematics(self, joint_angles):
    """
    joint_angles- [theta1, theta2, theta3, theta4] in radians
                for base_rotate, joint1, joint2, and joint3
    """
    self.fk_status = 0
    self.fk_result = (None, None)

    tries = 0
    while self.fk_status != KinematicsCalculator.STATUS_DONE \
       and not rospy.is_shutdown() and tries < KinematicsCalculator.MAX_FK_TRIES:
       tries += 1
       rospy.sleep(0.05)

    if tries == KinematicsCalculator.MAX_FK_TRIES:
      rospy.logwarn("Forward kinematics calc timed out. Are you publishing to /joint_states?")

    return self.fk_result

def main():
  """
  Main Script
  """
  #group = moveit_commander.MoveGroupCommander('arm')


  kinematics = KinematicsCalculator("arm")
  pos, rotation = kinematics.forward_kinematics([1, 1, 1, 1])
  print('fk result', pos, rotation)
  if pos is not None and rotation is not None:
    print('ik result', kinematics.inverse_kinematics(pos, rotation))
  else:
    print('Cannot run ik because fk failed!')
  rospy.spin()

if __name__ == '__main__':
  rospy.init_node('moveit_node')
  main()