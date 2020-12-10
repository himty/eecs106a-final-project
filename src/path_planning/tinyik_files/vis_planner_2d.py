import matplotlib.pyplot as plt
from matplotlib.widgets import Button
import tinyik
import numpy as np
import sys
sys.path.insert(0, "/Users/himty/Desktop/Serious_Stuff/UC_Berkeley/3_Junior/EECS 106A/eecs106a-final-project/src/path_planning/moveit_planner")
from next_pt_planner import NextPointPlanner

class OnClick2D():
    def __init__(self, planner):
        self.planner = planner 

        ##### ROBOT STUFF ####

        self.ee_pos = None
        self.obj_pos = None

        self.base_height = 7.5
        self.l1 = 10
        self.l2 = 12

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
            ])

        self.plan_pts = []
        self.ee_pts = []
        self.obj_pts = []

        self.default_x = 0
        self.default_y = 0
        self.default_z = 7.5

        self.near_pos = np.array([float('inf') for i in range(3)])
        self.far_pos = np.array([float('inf') for i in range(3)])

        ##### PLOT STUFF ####

        self.lims = [30, 30, 30]
        self.plt_dim = (1, 3)
        self.fig, self.ax = plt.subplots(1, 3, figsize=(13, 4))
        for axis in range(3):
            self.ax[axis].set_xlim([-self.lims[axis], self.lims[axis]])
            self.ax[axis].set_ylim([-self.lims[axis], self.lims[axis]])

        self.fig.canvas.mpl_connect('button_press_event', self.axis_onclick)

        plt.suptitle('Click where the end effector should go')

        # X squash
        self.ax[0].title.set_text("X Into Screen (Side View)")
        self.ax[0].scatter(0, 0, label='Origin')
        self.ax[0].plot(*zip([0, 0], [0, self.base_height]))
        self.ax[0].legend()
        self.ax[0].set_xlabel('y')
        self.ax[0].set_ylabel('z')

        # Y squash
        self.ax[1].title.set_text("Y Out of Screen (Side View)")
        self.ax[1].scatter(0, 0, label='Origin')
        self.ax[1].plot(*zip([0, 0], [0, self.base_height]))
        self.ax[1].legend()
        self.ax[1].set_xlabel('x')
        self.ax[1].set_ylabel('z')

        # Z squash
        self.ax[2].title.set_text("Top-Down View")
        self.ax[2].scatter(0, 0, label='Origin')
        self.ax[2].legend()
        self.ax[2].set_xlabel('x')
        self.ax[2].set_ylabel('y')

        self.axrandobj = plt.axes([0.93, 0.3, 0.065, 0.075])
        self.brandobj = Button(self.axrandobj, "Rand Obj")
        self.brandobj.on_clicked(self.rand_obj_onclick)

        self.axobjreset = plt.axes([0.93, 0.2, 0.065, 0.075])
        self.bobjreset = Button(self.axobjreset, "Reset Obj")
        self.bobjreset.on_clicked(self.obj_reset_onclick)

        self.axfarsim = plt.axes([0.93, 0.1, 0.065, 0.075])
        self.bfarsim = Button(self.axfarsim, 'Go Further')
        self.bfarsim.on_clicked(self.simulate_far_onclick)

        self.axnearsim = plt.axes([0.93, 0, 0.065, 0.075])
        self.bnearsim = Button(self.axnearsim, "Go Closer")
        self.bnearsim.on_clicked(self.simulate_near_onclick)

        self.colors = {
            'ee': 'r',
            'obj': 'b',
            'far': 'g',
            'near': 'm'
        }

        self.done = True

    def rand_obj_onclick(self, event):
        if self.ee_pos is None:
            return

        self.obj_pos = self.ee_pos + np.random.uniform(-7, 7, size=(3))
        [p.remove() for p in self.obj_pts]
        [p.remove() for p in self.plan_pts]
        self.obj_pts = []
        self.plan_pts = []

        self.plot_planner_pts()


        for axis in range(3):
            self.ax[axis].legend()

        plt.pause(0.1)

    def obj_reset_onclick(self, event):
        if self.obj_pos is None:
            return

        self.obj_pos = None
        [p.remove() for p in self.obj_pts]
        [p.remove() for p in self.plan_pts]
        self.obj_pts = []
        self.plan_pts = []

        for axis in range(3):
            self.ax[axis].legend()

        plt.suptitle('Next, click where the object should go')
        plt.pause(0.1)

    def simulate_near_onclick(self, event):
        self.simulate_nearfar_onclick(event, is_near=True)

    def simulate_far_onclick(self, event):
        self.simulate_nearfar_onclick(event, is_near=False)

    def simulate_nearfar_onclick(self, event, is_near):
        if not self.done:
            return

        if self.ee_pos is None or self.obj_pos is None:
            print("WARNING: Cannot simulate because end effector pos or object pos are not set. Click the plot")
            return

        if not is_near:
            if self.far_pos is None:
                return

            self.ee_pos = self.far_pos
        else:
            if self.near_pos is None:
                return

            self.ee_pos = self.near_pos

        [p.remove() for p in self.ee_pts]
        [p.remove() for p in self.obj_pts]
        [p.remove() for p in self.plan_pts]
        self.ee_pts = []
        self.obj_pts = []
        self.plan_pts = []
        self.plot_ee_pts()
        self.plot_planner_pts()


    def sim_mode_onclick(self, event):
        if self.bmode.label.get_text() == self.far_mode_label:
            self.bmode.label.set_text(self.near_mode_label) 
        else:
            self.bmode.label.set_text(self.far_mode_label)

    def plot_ee_pts(self):
        if not self.done:
            return

        self.done = False
        for axis in range(3):
            squashed_idxs = [i for i in range(3) if i != axis]
            if axis != 2: # not z
                self.ee_pts.append(self.ax[axis].plot(*zip([0, self.base_height],  self.ee_pos[squashed_idxs]), linestyle='-')[0])
            else:
                self.ee_pts.append(self.ax[axis].plot(*zip([0, 0],  self.ee_pos[squashed_idxs]), linestyle='-')[0])
            self.ee_pts.append(self.ax[axis].scatter(*self.ee_pos[squashed_idxs], label='End Effector', color=self.colors['ee']))
            self.ax[axis].legend()

        self.done = True

    def plot_planner_pts(self):
        if self.near_pos is None or self.far_pos is None:
            return 
        if not self.done:
            return

        self.done = False
        print('-')
        self.near_pos = self.planner.get_near_point(self.ee_pos, self.obj_pos)
        print("Near pos:", self.near_pos)
        if self.near_pos is not None:
            self.arm.ee = self.near_pos
            print("Near pos's joint angles:", self.arm.angles)
        else:
            print("WARNING: near_pos returned None for ee_pos {} and obj_pos {}".format(self.ee_pos, self.obj_pos))

        self.far_pos = self.planner.get_far_point(self.ee_pos, self.obj_pos)
        print("Far pos:", self.far_pos)
        if self.far_pos is not None:
            self.arm.ee = self.far_pos
            print("Far pos's joint angles:", self.arm.angles)
        else:
            print("WARNING: far_pos returned None for ee_pos {} and obj_pos {}".format(self.ee_pos, self.obj_pos))

        for axis in range(3):
            squashed_idxs = [i for i in range(3) if i != axis]
            self.obj_pts.append(self.ax[axis].scatter(*self.obj_pos[squashed_idxs], label='Object', color=self.colors['obj']))
            
            if self.near_pos is not None:
                self.plan_pts.append(self.ax[axis].scatter(*self.near_pos[squashed_idxs], label='Near', color=self.colors['near']))
                self.plan_pts.append(self.ax[axis].plot(*zip(self.ee_pos[squashed_idxs], self.near_pos[squashed_idxs]), linestyle=':')[0])
        
            if self.far_pos is not None:
                self.plan_pts.append(self.ax[axis].scatter(*self.far_pos[squashed_idxs], label='Far', color=self.colors['far']))
                self.plan_pts.append(self.ax[axis].plot(*zip(self.ee_pos[squashed_idxs], self.far_pos[squashed_idxs]), linestyle=':')[0])

            self.ax[axis].legend()
        plt.pause(0.1)
        self.done = True


    def axis_onclick(self, event):
        axis_idx = -1
        for axis in range(3):
            if self.ax[axis] == event.inaxes:
                axis_idx = axis
                break

        ax_pos = self.ax[axis_idx].transAxes.inverted().transform((event.x, event.y))
        ax_pos = [min(1, max(0, coord))*self.lims[axis]*2-self.lims[axis] for axis, coord in enumerate(ax_pos)]

        if axis_idx == 0: # x is squished
            pt = np.array([self.default_x, ax_pos[0], ax_pos[1]])
        elif axis_idx == 1: # y is squished
            pt = np.array([ax_pos[0], self.default_y, ax_pos[1]])
        elif axis_idx == 2:
            pt = np.array([ax_pos[0], ax_pos[1], self.default_z])
        else:
            return # Probably a click to the button

        if self.ee_pos is None:
            self.ee_pos = np.array(pt)
            print('Got end effector pos:', self.ee_pos)

            # Display for all three angles
            self.plot_ee_pts()

            plt.suptitle('Next, click where the object should go')
            plt.pause(0.1)
        elif self.obj_pos is None:
            # Hardcoded z coordinate
            self.obj_pos = np.array(pt)
            print('Got obj pos:', self.obj_pos)

            self.plot_planner_pts()

            plt.suptitle('Plotted planner points. Click the figure again to clear the screen')
            plt.pause(0.1)
        else:
            [p.remove() for p in self.ee_pts]
            [p.remove() for p in self.obj_pts]
            [p.remove() for p in self.plan_pts]
            self.ee_pts = []
            self.obj_pts = []
            self.plan_pts = []
            for axis in range(3):
                self.ax[axis].legend()
            plt.suptitle('Click where the end effector should go')
            plt.pause(0.1)

            self.ee_pos = None
            self.obj_pos = None
            self.near_pos = np.array([float('inf') for i in range(3)])
            self.far_pos = np.array([float('inf') for i in range(3)])

            print('Cleared.')
            print('\n---------------------\n')

if __name__ == "__main__":
    planner = NextPointPlanner()

    vis = OnClick2D(planner)

    plt.show()