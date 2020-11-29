import numpy as np
from lab3_skeleton import *
import tinyik

# TODO wrap this in a ROS node

class AlgPathPlanner():
    def __init__(self, max_dist_per_timestep=2, joint_angle_ranges=[]):
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

        # 'z' and 'x' are rotation axes.
        # Each array is the length of the joint between rotation axes
        # I don't think this library supports prismatic joints? 
        self.arm = tinyik.Actuator([
                'z',
                [0, 0, self.base_height],
                'x',
                [0, self.l1, 0],
                'x',
                [0, self.l2, 0],
                'x',
                [0, self.l3, 0]
            ])

    def get_far_point(self, arm_homog, obj_pos):
        return self._get_close_or_far_point(arm_homog, obj_pos, get_closer=False)

    def get_near_point(self, arm_homog, obj_pos):
        return self._get_close_or_far_point(arm_homog, obj_pos, get_closer=True)

    def _get_close_or_far_point(self, arm_pos, obj_pos, get_closer):
        """
            Draw a line between the obj and the end effector. The distance from the obj in the next timestep is
            maximized when you travel backwards along that line and minimized if you travel along it.

            arm_pos- [x, y, z] in base coordinates; numpy array
            cur_obj_pos- [x, y, z] in base coordinates; numpy array
            get_closer- boolean. Whether the resulting position should be closer or further from the obj_pos

            Returns:
            target_pos- [x, y, z] in base coordinates for where the end effector should go next
                        The position should be maximally closer or farther from obj_pos
                        Rotation is directly recoverable from position for this robot, so not going to encode that
        """
        dist_to_obj = np.linalg.norm(arm_pos - obj_pos)
        if get_closer and dist_to_obj < self.max_dist_per_timestep:
            return obj_pos
        else:
            # Law of cosines is
            # c^2 = a^2 + b^2 - 2*a*b*cos(angle opposite to C)
            # where a and b surround <angle opposite to C>

            # Want to calculate rotations about the first x-axis rotary joint
            offset = np.array([0, 0, self.base_height])
            rel_arm_pos = arm_pos - offset
            rel_obj_pos = obj_pos - offset

            a = np.linalg.norm(rel_arm_pos)

            # angle between <origin to end effector> and <origin to obj>
            c_angle = np.arccos(np.dot(rel_arm_pos, rel_obj_pos) / np.linalg.norm(rel_arm_pos) / np.linalg.norm(rel_obj_pos))

            # Solve for b in Solve for b in b^2 - 2*a*cos(angle opposite to C)*b + (a^2 - c^2) = 0
            # Do this for many possibilities for c and pick one that fits the maximum distance
            # constraint and is pretty far from the arm's starting point
            def get_radicand(c):
                # First half of the calculation for getting a target position
                a_ = 1
                b_ = -2*a*np.cos(c_angle)
                c_ = a**2 - c**2

                # We don't get an imaginary solution
                radicand = (b_**2 - 4*a_*c_)
                return radicand if radicand >= 0 else None
            def get_sol(radicand):
                # Complete the calculation in the previous function
                a_ = 1
                b_ = -2*a*np.cos(c_angle)

                # quadratic formula
                b1 = (-b_ + np.sqrt(radicand)) / (2*a_)
                b2 = (-b_ - np.sqrt(radicand)) / (2*a_)

                # b1 and b2 are magnitudes. They lie along the line from origin to the obj
                if get_closer:
                    target_pos = b1 * (rel_obj_pos / np.linalg.norm(rel_obj_pos))
                else:
                    target_pos = b2 * (rel_obj_pos / np.linalg.norm(rel_obj_pos))
                return target_pos

            target_pos = None
            # Get the pos with maximum distance away from the current arm position
            for c in np.arange(self.max_dist_per_timestep, 0, -0.1):
                radicand = get_radicand(c)
                if radicand is not None:
                    target_pos = get_sol(radicand) + offset
                    break
            
            if target_pos is None:
                # Previous sweep failed. Now sweep for distances that are out of range
                # Keep values close to the range
                for c in np.arange(self.max_dist_per_timestep, self.max_dist_per_timestep*5, 0.1):
                    radicand = get_radicand(c)
                    if radicand is not None:
                        target_pos = get_sol(radicand) + offset
                        # Scale down the solution to get a distance that's in range
                        target_pos = (target_pos - arm_pos) * (self.max_dist_per_timestep / c) + arm_pos
                        break

            return target_pos

    def forward_kinematics(self, thetas):
        """
            thetas- [theta1, theta2, ...] in radians
        """
        self.arm.angles = thetas
        return self.arm.ee

    def inverse_kinematics(self, pos):
        """
            Implements inverse kinematics for our arm :D

            pos- [x, y, z] in cm, relative to the base coordinates
            Returns thetas- [theta1, theta2, ...] in radians
        """
        # I love this library's api omg
        self.arm.ee = pos
        return self.arm.angles
