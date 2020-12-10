import numpy as np

class NextPointPlanner():
    def __init__(self, max_dist_per_timestep=2, move_const=10, z_boost=0.1, xy_boost=0.2):
        """
        Uses a handcoded algorithm to plan next point. No machine learning involved.
        When functions are called in succession, the resulting points will hopefully
        either move the arm further and further from the command sphere or closer and closer

        Everything is in cm and radians
        Looking straight out of the camera on the arm,
            - x is to the right
            - y is forwards
            - z is up

        Higher move_const tends to prioritize close or far movement over facing the object
        """
        # Can control the tradeoff between centering the ball in view vs getting closer/farther from it
        self.max_dist_per_timestep = max_dist_per_timestep
        self.move_const = move_const
        self.z_boost = z_boost
        self.xy_boost = xy_boost
        self.base_height = 7.5
        self.l1 = 10
        self.l2 = 12
        self.max_mag = self.l1+self.l2-2
        self.min_mag = np.sqrt(self.l1**2+self.l2**2)+1
        self.offset = np.array([0, 0, self.base_height]) # position of first joint

    def in_arm_range(self, arm_pos, obj_pos, get_closer):
        """
        keep distances relative to the first joint of the robot small
        and in ranges that the robot can actually reach
        """
        # can't go too far out or too close to the first joint
        rel_arm_pos = arm_pos - self.offset
        mag = np.linalg.norm(rel_arm_pos)
        unit = rel_arm_pos / mag

        # Need to turn instead of just clipping
        print('unit before', unit)
        # if mag < self.min_mag + 1:
        #     if not np.isclose(unit[2], 0, atol=1):
        #         print('z boost')
        #         # Change z to lift upwards
        #         if get_closer:
        #             if arm_pos[2] > obj_pos[2]:
        #                 unit[2] -= self.z_boost
        #             else:
        #                 unit[2] += self.z_boost
        #         else:
        #             if arm_pos[2] > obj_pos[2]:
        #                 unit[2] += self.z_boost
        #             else:
        #                 unit[2] -= self.z_boost
        #     else:
        #         print('xy boost')
        #         # TODO opposite side of the middle base determines direction 
        #         boost_dir = (obj_pos - self.offset)[:2]
        #         boost_dir /= np.linalg.norm(boost_dir)
        #         if get_closer:
        #             unit[0] += boost_dir[0] * self.xy_boost
        #             unit[1] += boost_dir[1] * self.xy_boost
        #         else:
        #             unit[0] -= boost_dir[0] * self.xy_boost
        #             unit[1] -= boost_dir[1] * self.xy_boost

            # boost or decrease the component to turn
            
            # print('unit after', unit)
            # unit = unit / np.linalg.norm(unit)

        clipped = np.clip(mag, self.min_mag, self.max_mag)*unit + self.offset

        # can't bend below a certain point, which is z
        clipped[2] = np.clip(clipped[2], self.base_height-self.l2, None)

        return clipped

    def get_far_point(self, arm_pos, obj_pos):
        """
        Get target end effector position that's farther from the object than before
        using absolute coordinates, relative to the true base frame
        """
        return self._get_close_or_far_point_no_face_obj(arm_pos, obj_pos, get_closer=False)

    def get_near_point(self, arm_pos, obj_pos):
        """
        Get target end effector position that's closer to the object than before
        using absolute coordinates, relative to the initial base frame
        """
        return self._get_close_or_far_point_face_obj(arm_pos, obj_pos, get_closer=True)

    def _get_far_point_at_obj(self, arm_pos, obj_pos):
        """
        Handle the case when the arm is at the object and we want to go further from it
        Go towards the first joint position
        """
        get_closer = False
        to_obj = obj_pos - self.offset
        to_obj = to_obj / np.linalg.norm(to_obj)

        target_pos = arm_pos - self.max_dist_per_timestep*to_obj
        return self.in_arm_range(target_pos, obj_pos, get_closer)

    def _get_close_or_far_point_no_face_obj(self, arm_pos, obj_pos, get_closer):
        """
        Don't care about facing the object 
        """
        # will give divide by zero for to_obj
        if np.all(np.isclose(arm_pos, obj_pos)):
            return self._get_far_point_at_obj(arm_pos, obj_pos)

        rel_arm_pos = arm_pos - self.offset
        rel_obj_pos = obj_pos - self.offset

        to_obj = rel_obj_pos - rel_arm_pos
        to_obj = to_obj / np.linalg.norm(to_obj)

        if get_closer:
            target_pos = arm_pos + self.max_dist_per_timestep*to_obj
        else:
            target_pos = arm_pos - self.max_dist_per_timestep*to_obj

        return self.in_arm_range(target_pos, obj_pos, get_closer)

    def _get_close_or_far_point_face_obj(self, arm_pos, obj_pos, get_closer):
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

        #### BASE CASES ####

        if get_closer and dist_to_obj < self.max_dist_per_timestep:
            return self.in_arm_range(obj_pos, obj_pos, get_closer)
        if not get_closer and np.all(np.isclose(arm_pos, obj_pos)):
            # Special case because relative rel_obj_pos has a 0 magnitude. go to the simple function
            return self._get_far_point_at_obj(arm_pos, obj_pos)

        # Want to calculate rotations relative to the first x-axis rotary joint
        rel_arm_pos = arm_pos - self.offset
        rel_obj_pos = obj_pos - self.offset

        rel_arm_pos_unit = rel_arm_pos/np.linalg.norm(rel_arm_pos)
        rel_obj_pos_unit = rel_obj_pos/np.linalg.norm(rel_obj_pos)

        if np.all(np.isclose(rel_arm_pos_unit, rel_obj_pos_unit, atol=0.05)) \
                or np.all(np.isclose(-rel_arm_pos_unit, rel_obj_pos_unit, atol=0.05)):
            # This type of input will give no solutions if passed into the else case
            # The arm is lined up with the object relative to the first joint
            return self._get_close_or_far_point_no_face_obj(arm_pos, obj_pos, get_closer)

        #### MAIN LOGIC ####
        # Law of cosines is
        # c^2 = a^2 + b^2 - 2*a*b*cos(angle opposite to C)
        # where a and b surround <angle opposite to C>

        a = np.linalg.norm(rel_arm_pos)

        # angle between <self.offset origin to end effector> and <self.offset origin to obj>
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
            b1 = (-b_ + np.sqrt(radicand)) / (2*a_)
            b2 = (-b_ - np.sqrt(radicand)) / (2*a_)

            # b1 and b2 are magnitudes. They lie along the line from self.offset origin to the self.offset obj
            if get_closer:
                if np.linalg.norm(rel_obj_pos) > np.linalg.norm(rel_arm_pos):
                    return b1 * (rel_obj_pos / np.linalg.norm(rel_obj_pos))
                else:
                    return b2 * (rel_obj_pos / np.linalg.norm(rel_obj_pos))
            else:
                if np.linalg.norm(rel_obj_pos) > np.linalg.norm(rel_arm_pos):
                    return b2 * (rel_obj_pos / np.linalg.norm(rel_obj_pos))
                else:
                    return b1 * (rel_obj_pos / np.linalg.norm(rel_obj_pos))

        # Find a target position
        # Sweep until the theoretical travel distance is large enough to give a real solution
        target_pos = None
        for c in np.arange(self.move_const, self.move_const*100, 0.1):
            radicand = get_radicand(c)
            if radicand is not None:
                target_pos = get_sol(radicand) + self.offset
                # Scale down the solution to get a distance that's within the true max dist per timestep
                target_pos = (target_pos - arm_pos) * (self.max_dist_per_timestep / c) + arm_pos
                break

        if target_pos is not None:
            return self.in_arm_range(target_pos, obj_pos, get_closer)
        else:
            print('wot returned None for ', arm_pos, obj_pos)
            return None
