import numpy as np
from lab3_skeleton import *

# TODO wrap this in a ROS node

class AlgPathPlanner():
    def __init__(self, max_dist_per_timestep, joint_angle_ranges):
        """
        Algorithmic path planner that uses a handcoded algorithm for planning. No machine learning involved.

        Everything is in cm and radians
        """
        self.max_dist_per_timestep = max_dist_per_timestep
        # TODO get this from a parameter server
        # [[theta1_min, theta1_max], [theta2_min, theta2_max], ...]
        self.joint_angle_ranges = joint_angle_ranges 

        # TODO hardcoded but get this from a parameter server
        self.base_height = 5

        # Initialize robot params
        self.xis, self.g0 = self.get_joint_twists()

    def get_far_point(self, cur_arm_pos, cur_block_pos):
        """
            Draw a line between the block and the end effector. The distance from the block in the next timestep is
            maximized when you travel backwards along that line

            cur_arm_pos- [x, y, z] in base coordinates; numpy array
            cur_block_pos- [x, y, z] in base coordinates; numpy array

            Returns:
            target_pos- position farther from the cur_block_pos
            target_rot- rotation matrix for making the end effector face the block
        """
        # TODO cap this value to something within the reachable workspace
        v = cur_arm_pos - cur_block_pos
        v = v / np.linalg.norm(v)
        target_pos = cur_arm_pos + v * self.max_dist_per_timestep
        target_rot = None # TODO! maybe get roll, pitch and yaw then create the rotation matrix
        return target_pos, target_rot

    def get_near_point(self, cur_arm_pos, cur_block_pos):
        """
            Differs from self.get_far_point() by a negative sign

            cur_arm_pos- [x, y, z] in base coordinates; numpy array
            cur_block_pos- [x, y, z] in base coordinates; numpy array

            Returns:
            target_pos- position closer to the cur_block_pos
            target_rot- rotation matrix for making the end effector face the block
        """
        # TODO cap this value to something within the reachable workspace
        v = cur_arm_pos - cur_block_pos
        v = v / np.linalg.norm(v)
        target_pos = cur_arm_pos - v * self.max_dist_per_timestep
        target_rot = None # TODO! maybe get roll, pitch and yaw then create the rotation matrix
        return target_pos, target_rot

    def get_joint_twists(self):
        q = np.ndarray((3,8))
        w = np.ndarray((3,7))
        
        ### TODO Make these very accurate
        # TODO more importantly, put the actual joint info here
        q[0:3,0] = [0.0635, 0.2598, 0.1188]
        q[0:3,1] = [0.1106, 0.3116, 0.3885]
        q[0:3,2] = [0.1827, 0.3838, 0.3881]
        q[0:3,3] = [0.3682, 0.5684, 0.3181]
        q[0:3,4] = [0.4417, 0.6420, 0.3177]
        q[0:3,5] = [0.6332, 0.8337, 0.3067]
        q[0:3,6] = [0.7152, 0.9158, 0.3063]
        q[0:3,7] = [0.7957, 0.9965, 0.3058]

        w[0:3,0] = [-0.0059,  0.0113,  0.9999]
        w[0:3,1] = [-0.7077,  0.7065, -0.0122]
        w[0:3,2] = [ 0.7065,  0.7077, -0.0038]
        w[0:3,3] = [-0.7077,  0.7065, -0.0122]
        w[0:3,4] = [ 0.7065,  0.7077, -0.0038]
        w[0:3,5] = [-0.7077,  0.7065, -0.0122]
        w[0:3,6] = [ 0.7065,  0.7077, -0.0038]

        R = np.array([[0.0076, 0.0001, -1.0000],
                      [-0.7040, 0.7102, -0.0053],
                      [0.7102, 0.7040, 0.0055]]).T

        g0 = np.hstack((R, q[0:3,7].reshape(3,1)))
        g0 = np.vstack((g0, np.zeros(4)))
        g0[3,3] = 1

        # write the twist xi_i for each joint in the manipulator
        xis = np.ndarray((6, 7))
        for i in range(7): 
            xis[:,i] = get_xi(w[:,i], q[:,i])

        return xis, g0

    def forward_kinematics(self, thetas):
        """
            Forward kinematics, which is essentially copied from Lab 3
            This is only used to check the inverse kinematics

            thetas- [theta1, theta2, ...] in radians

            Returns the xyz coordinates only
        """
        g = prod_exp(self.xis, thetas)
        return g.dot(self.g0)[:3,3]

    def inverse_kinematics(self, x, y, z):
        """
            Implements inverse kinematics for our arm :D

            x, y, and z are in cm, relative to the base coordinates
            Returns 
        """
        # TODO
        pass

