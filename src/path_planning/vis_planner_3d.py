import tinyik
import numpy as np
import open3d as o3d
from path_planner import AlgPathPlanner

# Open3d likes to be unblocked :/
# This script puts keyboard input on another thread
import threading

import argparse

# From https://stackoverflow.com/a/57387909/5901346
class KeyboardThread(threading.Thread):

    def __init__(self, render_links, name='keyboard-input-thread'):
        self.render_links = render_links

        # stages are get_ee_pos, get_obj_pos
        self.stage = 'get_ee_pos'
        self.stage2query = {
            'get_ee_pos': 'Enter end effector position as "x y z": ',
            'get_obj_pos': 'Enter object position as "x y z": ',
        }

        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window(window_name='tinyik vizualizer', width=640, height=480)

        self.obj_pos = []
        self.sphere_r = .12 # radius of circles
        self.ee_color = {'name': 'white', 'code': [.2, .2, .2]}
        self.obj_color = {'name': 'red', 'code': [.8, .2, .2]}
        self.near_color = {'name': 'green', 'code': [.1, .8, .1]}
        self.far_color = {'name': 'blue', 'code': [.1, .1, .8]}

        self.planner = AlgPathPlanner(2, [])

        # Draw some initial things
        axes = o3d.geometry.TriangleMesh.create_coordinate_frame(
                size=0.6, origin=[0, 0, 0])
        self.vis.add_geometry(axes)
        self.render_arm()
        
        super(KeyboardThread, self).__init__(name=name)
        self.start()

    def run(self):
        while True:
            self.input_cbk(input('Type "reset" to reset view or ' + self.stage2query[self.stage])) #waits to get input + Return

    def input_cbk(self, inp):
        if self.stage == 'get_ee_pos':
            if len(inp.strip().split(' ')) != 3:
                # Tell user to try again
                return
            else:
                ee_pos = [float(coord) for coord in inp.split(' ')]

            self.planner.arm.ee = ee_pos

            self.vis.clear_geometries()
            axes = o3d.geometry.TriangleMesh.create_coordinate_frame(
                size=0.6, origin=[0, 0, 0])
            self.vis.add_geometry(axes)
            self.render_arm()
            self.stage = 'get_obj_pos'
        elif self.stage == 'get_obj_pos':
            if len(inp.strip().split(' ')) != 3:
                # Tell user to try again
                return
            else:
                self.obj_pos = [float(coord) for coord in inp.split(' ')]

            obj = tinyik.visualizer.create_sphere(self.obj_pos, r=self.sphere_r, color=self.obj_color['code'])
            self.vis.add_geometry(obj)

            near_pos = self.planner.get_near_point(self.planner.arm.ee, self.obj_pos)
            self.planner.arm.ee = near_pos
            self.render_arm()
            obj = tinyik.visualizer.create_sphere(near_pos, r=self.sphere_r, color=self.near_color['code'])
            self.vis.add_geometry(obj)

            far_pos = self.planner.get_far_point(self.planner.arm.ee, self.obj_pos)
            self.planner.arm.ee = far_pos
            self.render_arm()
            obj = tinyik.visualizer.create_sphere(far_pos, r=self.sphere_r, color=self.far_color['code'])
            self.vis.add_geometry(obj)

            print('Displayed. {} ball is object, {} ball is near, {} ball is far'.format(
                        self.obj_color['name'], self.near_color['name'], self.far_color['name']))

            self.stage = 'get_ee_pos'
        else:
            raise ValueError('Unknown stage {}'.format(self.stage))

    def render_arm(self):
        if self.render_links:
            geos = self.get_joint_geometries()
            for geo in geos:
                self.vis.add_geometry(geo)
        else:
            print('yo')
            ee_pos = self.planner.arm.ee
            ee = tinyik.visualizer.create_sphere(ee_pos, r=self.sphere_r, color=self.ee_color['code'])
            self.vis.add_geometry(ee)

    def get_joint_geometries(self):
        """
        Copy pasted tinyik.visualize() function but removed
        the blocking function at the end. It also now returns the geometries generated
        """
        
        # Setting some values so the code runs in this class
        actuator = self.planner.arm
        target = None

        root = None
        p = None
        joints = []
        for c in actuator.components:
            if hasattr(c, 'axis'):
                gc = tinyik.visualizer.Joint(c)
                joints.append(gc)
            else:
                gc = tinyik.visualizer.Link(c)

            if root is None:
                root = gc
                p = gc
            else:
                p.child = gc
                p = gc

        for j, a in zip(joints, actuator.angles):
            j.angle = a

        if target:
            geos = root.geo(link_color=[.5, .5, .5])
            actuator.ee = target
            for j, a in zip(joints, actuator.angles):
                j.angle = a
            geos += root.geo()
            geos += [tinyik.visualizer.create_sphere(target, r=self.sphere_r, color=[.8, .2, .2])]
        else:
            geos = root.geo()

        return geos

    # I don't know how to update an existing geometry
    # def update_joint_geometries(self, ee_pos):
    #     """
    #     Get joint geometries when the end effector is at ee_pos

    #     ee_pos- (x, y, z) target position of the end effector
    #     """
    #     self.planner.arm.ee = ee_pos

    #     # Assume the vis joints correspond to the first few geometries in self.geos
    #     mat = np.eye(4)
    #     for geo, vis_component in zip(self.geos, self.vis_components):
    #         print(type(geo), type(vis_component), type(vis_component.c))
    #         if type(vis_component) is tinyik.visualizer.Link:
    #             norm = np.linalg.norm(vis_component.c.coord)
    #             base = np.array([0., 0., norm])
    #             cross = np.cross(base, self.c.coord)
    #             axis = cross / np.linalg.norm(cross)
    #             angle = np.arccos(np.dot(base, self.c.coord) / (norm ** 2))
    #             geo.transform(mat @ tinyik.visualizer.rotate(axis, angle) @ translate(base / 2))
    #         elif type(vis_component) is tinyik.visualizer.Joint:
    #             rx = {
    #                 'x': [0., 1., 0.],
    #                 'y': [1., 0., 0.],
    #                 'z': [0., 0., 1.],
    #             }
    #             geo.transform(mat @ tinyik.visualizer.rotate(rx[vis_component.c.axis], np.pi / 2))
    #         else:
    #             raise ValueError('Unknown vis_component type {}! Did the order in self.geos get messed up?' \
    #                     .format(type(vis_component)))

    #         mat = mat @ vis_component.mat()

def visualize_3d(render_links):
    # Rendering updates are asynchronous in this thread
    kthread = KeyboardThread(render_links)
    while True:
        kthread.vis.poll_events()
        kthread.vis.update_renderer()

    # o3d.visualization.draw_geometries(
    #     geos, window_name='tinyik vizualizer', width=640, height=480)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--no_arm', action='store_false', default=True)
    args = parser.parse_args()

    visualize_3d(args.no_arm)