import numpy as np
from lab3_skeleton import *

# TODO wrap this in a ROS node

class AlgPathPlanner():
    def __init__(self, max_dist_per_timestep, joint_angle_ranges):
        """
        Algorithmic path planner that uses a handcoded algorithm for planning. No machine learning involved.

        Everything is in cm and radians
        Looking straight out of the camera on the arm,
            - x is to the right
            - y is forwards
            - z is up
        """
        self.max_dist_per_timestep = max_dist_per_timestep
        # TODO get this from a parameter server
        # [[theta1_min, theta1_max], [theta2_min, theta2_max], ...]
        self.joint_angle_ranges = joint_angle_ranges 

        # TODO hardcoded but get this from a parameter server
        self.base_height = 5
        self.l1 = 2
        self.l2 = 10
        self.l3 = 8

        # Initialize robot params
        self.xis, self.g0 = self.get_joint_twists()

    def get_far_point(self, arm_homog, block_pos):
        return self._get_close_or_far_point(arm_homog, block_pos, get_closer=False)

    def get_near_point(self, arm_homog, block_pos):
        return self._get_close_or_far_point(arm_homog, block_pos, get_closer=True)

    def _get_close_or_far_point(self, arm_homog, block_pos, get_closer):
        """
            Draw a line between the block and the end effector. The distance from the block in the next timestep is
            maximized when you travel backwards along that line and minimized if you travel along it.

            arm_homog- 4x4 homogenous matrix representing rotation and position of the end effector in base coordinates;
                        numpy array
            cur_block_pos- [x, y, z] in base coordinates; numpy array
            get_closer- boolean. Whether the resulting position should be closer or further from the block_pos

            Returns:
            target_homog- 4x4 matrix encoding the target rotation and position. 
                        The position should be maximally closer or farther from block_pos
                        The end effector should face the block
        """
        # TODO cap this value to something within the reachable workspace
        arm_R, arm_pos = arm_homog[:3, :3], arm_homog[:3, 3]

        # Normalized vector pointing from the arm to the block
        v = block_pos - arm_pos
        v = v / np.linalg.norm(v)

        # Get target position
        if get_closer:
            target_pos = arm_pos + v * self.max_dist_per_timestep
        else:
            target_pos = arm_pos - v * self.max_dist_per_timestep

        # Get target rotation, facing the block
        # I think this solution is more likely to be oriented upwards?
        # https://stackoverflow.com/a/42594173/5901346
        y = v
        x = np.cross(np.array([0,0,1]), y) # cross product with "up" and direction to block
        z = np.cross(x, v)
        target_rot = np.vstack([x, y, z])

        target_homog = get_3dto4d(target_rot, target_pos)
        return target_homog

    def get_joint_twists(self):
        q = np.ndarray((3,4))
        w = np.ndarray((3,4))
        
        ### TODO Make these very accurate
        # TODO more importantly, put the actual joint info here
        q[0:3,0] = [0, 0, 0]
        q[0:3,1] = [0, 0, self.base_height]
        q[0:3,2] = [0, self.l1, self.base_height]
        q[0:3,3] = [0, self.l1+self.l2, self.base_height]

        w[0:3,0] = [0, 0, 1]
        w[0:3,1] = [-1, 0, 0]
        w[0:3,2] = [-1, 0, 0]
        w[0:3,3] = [-1, 0, 0]

        R = np.eye(3) # Initially the same as the base frame
        q_init = np.array([0, self.l1+self.l2+self.l3, self.base_height])
        g0 = get_3dto4d(R, q_init)

        # write the twist xi_i for each joint in the manipulator
        xis = np.ndarray((6, 4))
        for i in range(4):
            xis[:,i] = get_xi(w[:,i], q[:,i])

        return xis, g0

    def forward_kinematics(self, thetas):
        """
            Forward kinematics, which is essentially copied from Lab 3
            This is only used to check the inverse kinematics

            thetas- [theta1, theta2, ...] in radians
        """
        g = prod_exp(self.xis, thetas)
        return g.dot(self.g0)

    def inverse_kinematics(self, x, y, z):
        """
            Implements inverse kinematics for our arm :D

            x, y, and z are in cm, relative to the base coordinates
            Returns 
        """
        # TODO
        pass

