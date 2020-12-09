import numpy as np

class NextPointPlanner():
    def __init__(self, max_dist_per_timestep=2, joint_angle_ranges=[]):
        """
        Uses a handcoded algorithm to plan next point. No machine learning involved.
        When functions are called in succession, the resulting points will hopefully
        either move the arm further and further from the command sphere or closer and closer

        Everything is in cm and radians
        Looking straight out of the camera on the arm,
            - x is to the right
            - y is forwards
            - z is up
        """
        # Can control the tradeoff between centering the ball in view vs getting closer/farther from it
        self.max_dist_per_timestep = max_dist_per_timestep
        self.base_height = 7.5
        self.l1 = 10
        self.l2 = 12

    def get_far_point(self, arm_pos, obj_pos):
        """
        Get target end effector position that's farther from the object than before
        using absolute coordinates, relative to the true base frame
        """
        return self._get_close_or_far_point(arm_pos, obj_pos, get_closer=False)

    def get_near_point(self, arm_pos, obj_pos):
        """
        Get target end effector position that's closer to the object than before
        using absolute coordinates, relative to the initial base frame
        """
        return self._get_close_or_far_point(arm_pos, obj_pos, get_closer=True)

    def _get_close_or_far_point(self, arm_pos, obj_pos, get_closer):
        """
            Get closer or farther from the ball, while also centering the ball in view.

            arm_pos- [x, y, z] in base coordinates; numpy array
            cur_obj_pos- [x, y, z] in base coordinates; numpy array
            get_closer- boolean. Whether the resulting position should be closer or further from the obj_pos

            Returns:
            target_pos- [x, y, z] in base coordinates for where the end effector should go next
                        The position should be maximally closer or farther from obj_pos
                        Rotation is directly recoverable from position for this robot, so not going to encode that
        """
        # TODO only return points that the arm can reach (defined in self.joint_angle_ranges)

        dist_to_obj = np.linalg.norm(arm_pos - obj_pos)
        if get_closer and dist_to_obj < self.max_dist_per_timestep:
            return obj_pos
        else:
            # Law of cosines is
            # c^2 = a^2 + b^2 - 2*a*b*cos(angle opposite to C)
            # where a and b surround <angle opposite to C>

            # Want to calculate rotations relative to the first x-axis rotary joint
            offset = np.array([0, 0, self.base_height])
            rel_arm_pos = arm_pos - offset
            rel_obj_pos = obj_pos - offset

            a = np.linalg.norm(rel_arm_pos)

            # angle between <offset origin to end effector> and <offset origin to obj>
            c_angle = np.arccos(np.dot(rel_arm_pos, rel_obj_pos) / np.linalg.norm(rel_arm_pos) / np.linalg.norm(rel_obj_pos))

            # Solve for b in b^2 - 2*a*cos(angle opposite to C)*b + (a^2 - c^2) = 0
            def get_radicand(c):
                # First half of the calculation for getting a target position
                a_ = 1
                b_ = -2*a*np.cos(c_angle)
                c_ = a**2 - c**2

                # We don't want an imaginary solution
                radicand = (b_**2 - 4*a_*c_)
                return radicand if radicand >= 0 else None
            def get_sol(radicand):
                # Complete the calculation in the previous function
                a_ = 1
                b_ = -2*a*np.cos(c_angle)
                # c_ is only used in radicand, which is given

                # quadratic formula
                b1 = min(self.l1+self.l2, max(0, (-b_ + np.sqrt(radicand)) / (2*a_)))
                b2 = min(self.l1+self.l2, max(0, (-b_ - np.sqrt(radicand)) / (2*a_)))

                # b1 and b2 are magnitudes. They lie along the line from offset origin to the offset obj
                if get_closer:
                    if np.linalg.norm(rel_obj_pos) > np.linalg.norm(rel_arm_pos):
                        target_pos = b1 * (rel_obj_pos / np.linalg.norm(rel_obj_pos))
                    else:
                        target_pos = b2 * (rel_obj_pos / np.linalg.norm(rel_obj_pos))
                else:
                    if np.linalg.norm(rel_obj_pos) > np.linalg.norm(rel_arm_pos):
                        target_pos = b2 * (rel_obj_pos / np.linalg.norm(rel_obj_pos))
                    else:
                        target_pos = b1 * (rel_obj_pos / np.linalg.norm(rel_obj_pos))
                return target_pos

            # Find a target position
            # Sweep until the theoretical travel distance is large enough to give a real solution
            target_pos = None
            for c in np.arange(self.max_dist_per_timestep, self.max_dist_per_timestep*5, 0.1):
                radicand = get_radicand(c)
                if radicand is not None:
                    target_pos = get_sol(radicand) + offset
                    # Scale down the solution to get a distance that's in range
                    target_pos = (target_pos - arm_pos) * (self.max_dist_per_timestep / c) + arm_pos
                    break

            return target_pos
