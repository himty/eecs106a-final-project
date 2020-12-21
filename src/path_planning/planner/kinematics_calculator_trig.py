#!/usr/bin/env python
import sys

import numpy as np
import traceback

from lab3_skeleton import *

class KinematicsCalculator():
  """
  An instance of this has callable functions that compute
  inverse kinematics and forward kinematics.

  No ROS requirements for forward or inverse kinematics
  """
  def __init__(self):
    self.base_height = 7.5
    self.l1 = 10
    self.l2 = 12

    # Initialize robot params
    self.xis, self.g0 = self.get_joint_twists()

    self.offset = np.array([0, 0, self.base_height])

  def inverse_kinematics(self, pos, prev_joint_angles=None):
    """
    Does some trigonometry with the robot's structure to get joint angles from end effector position

    pos- [x, y, z] end effector position, in cm
    prev_joint_angles- the joint angles in the last timestep, in degrees.
                    If this is None, the ik result may flip flop between 0 and 180 degrees in base angle

    Returns joint angles [theta1, theta2, theta3] in degrees
    """
    # Law of cosines for SSS triangles
    # cos(C) = (a^2 + b^2 - c^2) / (2*a*b)
    # joint1_init = arccos_quadrant(arm_projected.dot(y), np.linalg.norm(arm_projected) * 1)

    # Angle for the base angle, with everything projected to the xy plane
    base_angle = get_angle_2d(np.array([pos[1], pos[0]]), [1, 0])
    if prev_joint_angles is not None and not np.isclose(np.round((prev_joint_angles[0]*np.pi/180), decimals=2)%(2*np.pi), np.round(base_angle, decimals=2)%(2*np.pi), atol=np.pi/4):
        # Ensure the inverse kinematics doesn't flip to the other side by pi radians
        base_angle = base_angle - np.pi

    # Arm position relative to the first joint
    rel_arm_pos = in_2d_plane(pos - self.offset, base_angle=base_angle)
    rel_arm_mag = np.linalg.norm(rel_arm_pos)

    # Bend the second joint towards the base_angle side if it wants to go in the positive direction
    # This ensures that the arm always has this hunkering down shape
    bend_forward = rel_arm_pos[0] >= 0

    rel_init_pos = in_2d_plane(self.g0[:3, 3] - self.offset, positive=True)

    # Get angle for joint 1
    joint1_angle_top = safe_arccos((self.l1**2 + rel_arm_mag**2 - self.l2**2) / (2*self.l1*rel_arm_mag))
    joint1_angle_bot = get_angle_2d(rel_init_pos, rel_arm_pos)
    if joint1_angle_bot < 0 and rel_arm_pos[0] < 0:
        joint1_angle_bot += np.pi*2

    if bend_forward:
        joint1_angle = joint1_angle_top + joint1_angle_bot
    else:
        joint1_angle = joint1_angle_bot - joint1_angle_top

    # Get angle for joint 2
    joint2_angle_inv = safe_arccos((self.l1**2 + self.l2**2 - rel_arm_mag**2) / (2*self.l1*self.l2))
    if bend_forward:
        joint2_angle = -(np.pi - joint2_angle_inv)
    else:
        joint2_angle = np.pi - joint2_angle_inv

    base_norm = np.round(base_angle * 180/np.pi, decimals=2) % 360
    joint1_norm = np.round((joint1_angle) * 180/np.pi, decimals=2) % 360
    joint2_norm = np.round((joint2_angle) * 180/np.pi, decimals=2) % 360

    return np.array([base_norm, joint1_norm, joint2_norm])

  def get_joint_twists(self):
    """
    Initialize the robot geometry for use in forward kinematics

    Return xis - matrix of joint twists
           g0 - 4x4 homogenous matrix defining the robot's initial pose
    """

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
    joint_angles- [theta1, theta2, theta3] in degrees
                for base_rotate, joint1, and joint2

    Returns 4x4 homogenous matrix
    """
    joint_angles = np.array(joint_angles) * np.pi / 180

    # Code after this point should use radians
    g = prod_exp(self.xis, np.array(joint_angles))
    g_ee = g.dot(self.g0) # pose of end effector

    return g_ee

def rot_mat_to_euler(r): 
    """
    Equation is from Stack Overflow: https://stackoverflow.com/a/46460540/5901346

    r - 4x4 rotation matrix, numpy array

    Returns [roll, pitch, yaw] in degrees
    """
    if (r[0, 2] == 1) | (r[0, 2] == -1): # special case 
        e3 = 0 # set arbitrarily 
        dlt = np.arctan2(r[0, 1], r[0, 2]) 
        if r[0, 2] == -1: 
            e2 = np.pi/2 
            e1 = e3 + dlt 
        else: 
            e2 = -np.pi/2 
            e1 = -e3 + dlt 
    else: 
        e2 = -np.arcsin(r[0, 2]) 
        e1 = np.arctan2(r[1, 2]/np.cos(e2), r[2, 2]/np.cos(e2)) 
        e3 = np.arctan2(r[0, 1]/np.cos(e2), r[0, 0]/np.cos(e2)) 
    return np.array([e1, e2, e3]) * 180 / np.pi

def in_2d_plane(v, base_angle=None, positive=True):
    """
    Returns the 3d vector v as a 2d vector, flattened to the plane where the base is facing
    Alternatively, positive=True makes this always return a positive x coordinate

    v- 3d vector
    base_angle - angle of the base of the robot. Reference for where the positive direction is
    positive - if base_angle is None, this hardcodes whether this vector is facing the positive direction
    """
    v_angle = get_angle_2d(np.array([v[1], v[0]]), [1, 0])
    if (base_angle is not None and np.isclose(v_angle, base_angle, atol=np.pi/2)) \
            or (base_angle is None and positive):
        return np.array([np.sqrt(v[0]**2 + v[1]**2), v[2]])
    else:
        return np.array([-np.sqrt(v[0]**2 + v[1]**2), v[2]])

def get_angle_2d(x1, x2):
    """
    Returns angle between two 2d vectors
    https://math.stackexchange.com/a/879474/741663
    """
    dot = x1.dot(x2)
    det = x1[0]*x2[1] - x1[1]*x2[0]
    return np.arctan2(det, dot)  # atan2(y, x)

def safe_arccos(x):
    """
    An arccos function that shouldn't ever return NaN and should react nicely to NaN inputs
    """
    return np.arccos(np.clip(x, -1, 1)) if x != float('nan') else np.pi

def main():
  """
  Main Script
  """
  kinematics = KinematicsCalculator()

  import time
  
  starttime = time.time()
  g = kinematics.forward_kinematics([0, 0, 0])

  pos = g[:3, 3]
  rotation = rot_mat_to_euler(g)

  print('fk result', pos, rotation)
  print('fk time', time.time() - starttime)

  starttime = time.time()
  print('ik result', kinematics.inverse_kinematics(pos))
  print('ik time', time.time() - starttime)

  print('\nRunning correctness tests...')
  for base in np.linspace(-360, 360, 10):
    for joint1 in np.linspace(-360, 360, 10):
      for joint2 in np.linspace(-360, 360, 10):
        args = np.array([base, joint1, joint2])
        g = kinematics.forward_kinematics(args)
        pos = g[:3, 3]

        ik_result = kinematics.inverse_kinematics(pos, prev_joint_angles=[a-0.01 for a in [base, joint1, joint2]])

        g_again = kinematics.forward_kinematics(ik_result)
        pos_again = g_again[:3, 3]

        assert np.all(np.isclose(pos, pos_again, atol=0.01)), ("pos {} not close to expected pos {}. \n" \
                                + '\n\nInput: {} angles \n-- fk --> {} pos \n-- ik --> {} angles \n-- fk --> {} pos' \
                                + '\n\nFailed test') \
                                .format(pos, pos_again, [base, joint1, joint2], pos, ik_result, pos_again) 
                    
  print('Passed')


if __name__ == '__main__':
  main()