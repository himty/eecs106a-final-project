import numpy as np

# Pretty untested

class LearningEnv:
    def __init__(self, 
            env_bounds=[[-12, 12], [-12, 12], [0, 24]],
            joint_angle_ranges=[]# TODO
            ):
        # Everything is in inches and radians (?)

        # Assumes floor is at z=0 and no other obstacles are in the way
        # # [[x_min, x_max], [y_min, y_max], [z_min, z_max]]
        self._env_bounds = env_bounds 

        # [[theta1_min, theta1_max], [theta2_min, theta2_max], ...] for all joints
        # Mostly to check if any outputs will make the robot clip into itself
        self._joint_angle_ranges = joint_angle_ranges

        self._max_obj_dist = 12 # hardcoded?

    def get_train_data(n):
        # n is number of data points to return
        data = []
        for i in range(n):
            dist = [np.random.uniform(1, self._max_obj_dist)]
            joint_angles = [np.random.uniform(**a) for a in self._joint_angle_ranges]
            data.append(np.array(dist + joint_angles))
        return data

    def joints_not_clipping(joint_angles):
        return np.all([joint_angles[i] >= self._joint_angle_ranges[i][0] and joint_angles[i] <= self._joint_angle_ranges[i][1]
                    for i in range(len(self._joint_angle_ranges))])




